"""Check the laptop launch without needing ROS or car hardware."""
import os
import unittest
import xml.etree.ElementTree as ET


LAUNCH_DIR = os.path.join(os.path.dirname(__file__), "..", "launch")


class OfflineReplayLaunchTests(unittest.TestCase):
    def setUp(self):
        self.root = ET.parse(os.path.join(LAUNCH_DIR, "fault_injection", "manager_offline_replay.launch")).getroot()
        self.nodes = {node.get("name"): node for node in self.root.findall("node")}

    def params(self, name):
        return {p.get("name"): p.get("value") for p in self.nodes[name].findall("param")}

    def test_no_hardware_or_automatic_feature_startup(self):
        self.assertEqual([], self.root.findall(".//include"))
        self.assertEqual(set(self.nodes), {
            "steering_fault_proxy_recorded", "recorded_acceleration_fault_proxy",
            "sensor_fault_injection_manager", "test_configuration_monitor"})
        self.assertTrue(all(n.get("pkg") == "adeye" for n in self.nodes.values()))

    def test_replay_sources_and_single_actuator_output_owners(self):
        for name, axis in (("steering_fault_proxy_recorded", "steering"),
                           ("recorded_acceleration_fault_proxy", "acceleration")):
            params = self.params(name)
            self.assertEqual(params["input_topic"], "$(arg recorded_%s_topic)" % axis)
            self.assertEqual(params["output_topic"], "/%s_requested_phy" % axis)
            self.assertEqual(params["enable_goal_stop_priority"], "false")
            self.assertEqual(params["zero_on_source_loss"], "false")
            self.assertEqual(params["enable_fault_timeout"], "$(arg enable_physical_fault_timeout)")

    def test_sensor_processing_is_optional_and_uses_replay(self):
        args = {a.get("name"): a.get("default") for a in self.root.findall("arg")}
        self.assertEqual(args["enable_lidar_path"], "false")
        self.assertEqual(args["enable_gnss_path"], "false")
        self.assertEqual(args["lidar_input_topic"], "/replay/points_raw")
        self.assertEqual(args["gnss_input_topic"], "/replay/fix")
        self.assertIn("enable_lidar_path", self.nodes["sensor_fault_injection_manager"].get("if"))
        self.assertEqual(self.params("sensor_fault_injection_manager")["lidar_output_topic"], "/points_raw")

    def test_ranges_match_current_real_car_defaults(self):
        common = ET.parse(os.path.join(LAUNCH_DIR, "manager_real_world.launch")).getroot()
        defaults = {a.get("name"): a.get("default") for a in common.findall("arg")}
        for arg in self.root.findall("arg"):
            name = arg.get("name")
            if name.startswith("steering_proxy_") or name.startswith("recorded_acceleration_") and name != "recorded_acceleration_topic":
                self.assertEqual(arg.get("default"), defaults[name], name)

    def test_offline_settings_cannot_replace_car_startup_settings(self):
        params = self.params("test_configuration_monitor")
        self.assertEqual(params["profile"], "offline_replay")
        self.assertEqual(params["enable_startup_configuration_services"], "false")
        self.assertEqual(params["startup_configuration_enabled"], "false")

    def test_package_launch_includes_point_to_existing_files(self):
        package = os.path.abspath(os.path.join(LAUNCH_DIR, ".."))
        prefix = "$(find adeye)/"
        for folder, _, names in os.walk(LAUNCH_DIR):
            for name in names:
                if not name.endswith(".launch"):
                    continue
                filename = os.path.join(folder, name)
                for include in ET.parse(filename).getroot().iter("include"):
                    target = include.get("file", "")
                    if target.startswith(prefix) and "$(" not in target[len(prefix):]:
                        self.assertTrue(os.path.isfile(os.path.join(package, target[len(prefix):])),
                                        "%s includes missing %s" % (filename, target))

    def test_launch_basenames_are_unique_for_existing_console_commands(self):
        found = {}
        for folder, _, names in os.walk(LAUNCH_DIR):
            for name in names:
                if name.endswith(".launch"):
                    self.assertNotIn(name, found, "Duplicate launch filename: " + name)
                    found[name] = folder
        self.assertTrue(os.path.isfile(os.path.join(LAUNCH_DIR, "manager_real_world.launch")))
        self.assertEqual(found["manager_offline_replay.launch"], os.path.join(LAUNCH_DIR, "fault_injection"))


if __name__ == "__main__":
    unittest.main()
