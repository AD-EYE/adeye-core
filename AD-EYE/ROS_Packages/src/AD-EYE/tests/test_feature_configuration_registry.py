"""Check the feature list without starting ROS, the manager or CAN communication."""
import ast
import json
import os
import sys
import unittest
import xml.etree.ElementTree as ET

PACKAGE = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(PACKAGE, "src"))
import test_feature_registry as registry


class FeatureRegistryTests(unittest.TestCase):
    def setUp(self):
        self.data = registry.get_registry()
        self.features = {item["id"]: item for item in self.data["features"]}

    def test_registry_valid_json_and_no_applied_state_inference(self):
        self.assertTrue(registry.validate_registry(self.data))
        self.assertEqual(json.loads(json.dumps(self.data)), self.data)
        self.assertFalse(self.data["applies_configuration"])
        for feature in self.features.values():
            self.assertFalse(feature["default_hint_is_applied_state"])
            self.assertNotIn("applied", feature)
            self.assertNotIn("enabled", feature)

    def test_returned_metadata_is_independent(self):
        self.data["features"][0]["dependencies"].append("not-real")
        self.data["reset_policy"]["never_restore"].clear()
        self.assertTrue(registry.validate_registry(registry.get_registry()))

    def test_complete_fault_inventory_and_exclusions(self):
        faults = [item for item in self.features.values() if item["kind"] == "fault_availability"]
        self.assertEqual(len(faults), 29)
        commands = {item["command"] for item in faults}
        for command in ("STEERCONSTANT_command", "ACCELCONSTANT_command", "LIDAR_NOISE_STDDEV_M_command",
                        "LIDAR_KEEP_EVERY_N_command", "LIDAR_YAW_DEG_command", "GNSS_DROPOUT_command"):
            self.assertIn(command, commands)
        self.assertFalse(commands.intersection(self.data["excluded_controls"]))
        for feature in faults:
            self.assertEqual(feature["current_change_mode"], "not_supported")
            self.assertTrue(feature["activation_runtime_supported"])
            self.assertIsNone(feature["default_hint"])
            self.assertTrue(feature["dependencies"])

    def test_runtime_parameters_not_confused_with_feature_switches(self):
        for identifier in ("fault_timeout_duration", "fault_ranges"):
            self.assertEqual(self.features[identifier]["current_change_mode"], "runtime_idle_only")
        for identifier in ("steering_proxy", "acceleration_proxy", "steering_fault_processing",
                           "acceleration_fault_processing", "lidar_processing", "gnss_processing",
                           "goal_stop_action", "goal_monitor", "manual_recovery", "recording",
                           "fault_timeout_enabled", "recorded_steering", "recorded_acceleration"):
            self.assertEqual(self.features[identifier]["current_change_mode"], "manual_restart")
        for identifier in ("range_enforcement", "gui_direct_control_interlocks", "command_watchdog_actions"):
            self.assertEqual(self.features[identifier]["current_change_mode"], "code_change")

    def test_sensor_presence_not_pass_through_bypass(self):
        self.assertEqual(self.features["lidar_processing"]["planned_change_mode"], "manual_restart")
        self.assertEqual(self.features["lidar_fault_processing"]["current_change_mode"], "not_supported")
        self.assertIn("unchanged", self.features["lidar_fault_processing"]["disabled_behavior"])
        self.assertIn("no LiDAR relay", self.features["lidar_processing"]["disabled_behavior"])

    def test_reset_ownership_persistence_and_no_vehicle_actions(self):
        policy = self.data["reset_policy"]
        self.assertEqual(policy["restart"], "manual")
        self.assertEqual(policy["persistence"], "session_only_unless_explicitly_saved")
        self.assertIn("no requests", policy["no_changes"])
        self.assertIn("do not overwrite", policy["conflict"])
        self.assertIn("unknown", policy["missing_evidence"])
        self.assertIn("reset prepared", policy["restart_required"])
        for item in ("active_fault", "old_actuator_command", "pending_fault_timer", "ad_mode", "recovery_hold_release"):
            self.assertIn(item, policy["never_restore"])
        self.assertIn("recorded_bags", policy["never_touch"])
        self.assertIn("pending", policy["active_test"])

    def test_core_launch_default_hints_match_current_file(self):
        root = ET.parse(os.path.join(PACKAGE, "launch", "manager_real_world.launch")).getroot()
        defaults = {element.get("name"): element.get("default") for element in root.findall("arg")}
        for feature in self.features.values():
            for binding in feature["startup_bindings"]:
                prefix = "launch/manager_real_world.launch:arg:"
                if not binding.startswith(prefix):
                    continue
                name = binding[len(prefix):]
                self.assertIn(name, defaults)
                # Computed defaults must be resolved later, not guessed here.
                if defaults[name] in ("true", "false"):
                    self.assertEqual(feature["default_hint"], defaults[name] == "true", feature["id"])

    def test_profiles_reference_real_launches_without_invented_effective_defaults(self):
        self.assertEqual(len(self.data["profiles"]), 9)
        for profile in self.data["profiles"]:
            self.assertTrue(os.path.isfile(os.path.join(PACKAGE, "launch", profile["launch_file"])))
            self.assertNotIn("enabled_features", profile)
        self.assertIn("OR", self.features["goal_monitor"]["notes"])

    def test_local_source_references_exist(self):
        for feature in self.features.values():
            for reference in feature["references"]:
                if reference.startswith(("gui:", "ros2can:")):
                    continue  # Separate repositories are not required to run these tests.
                self.assertTrue(os.path.isfile(os.path.join(PACKAGE, reference)), reference)

    def test_bad_metadata_rejected(self):
        mutations = [lambda data: data["features"].append(data["features"][0]),
                     lambda data: data["features"][0]["dependencies"].append("unknown"),
                     lambda data: data["features"][0]["dependencies"].append(data["features"][0]["id"]),
                     lambda data: data["features"][0].update(default_hint_is_applied_state=True),
                     lambda data: data["features"][0].update(current_change_mode="pretend-live"),
                     lambda data: data.update(applies_configuration=True),
                     lambda data: data["reset_policy"].update(restart="automatic"),
                     lambda data: data["reset_policy"].update(never_restore=[])]
        for mutate in mutations:
            data = registry.get_registry()
            mutate(data)
            with self.assertRaises(ValueError):
                registry.validate_registry(data)

    def test_module_has_no_ros_io_process_or_import_side_effect_path(self):
        with open(registry.__file__) as source:
            tree = ast.parse(source.read())
        imports = [node.names[0].name for node in ast.walk(tree) if isinstance(node, ast.Import)]
        self.assertEqual(imports, ["copy"])
        forbidden = {"open", "exec", "eval", "print", "set_param", "publish", "callService", "Popen"}
        for node in ast.walk(tree):
            if isinstance(node, ast.Call):
                name = node.func.id if isinstance(node.func, ast.Name) else node.func.attr if isinstance(node.func, ast.Attribute) else ""
                self.assertNotIn(name, forbidden)


if __name__ == "__main__":
    unittest.main()
