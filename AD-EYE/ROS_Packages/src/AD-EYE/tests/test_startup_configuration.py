"""Next-launch drafts and files must never change the running vehicle."""
import json
import os
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from unittest import mock

PACKAGE = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(PACKAGE, "src"))
from startup_configuration import ARGUMENTS, DEFAULTS, FORMAT, MODULES, SETTINGS, StartupStore, StartupServices, read_file, validate
from launch_configured_manager import launch_arguments


class StartupConfigurationTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.path = os.path.join(self.temporary.name, "owned", "startup.json")
        self.store = StartupStore(self.path, DEFAULTS)

    def change(self, operation, key=None, value=None):
        return self.store.change(self.store.revision, operation, key, value)

    def test_default_status_is_read_only_and_does_not_create_directory(self):
        status = self.store.status()
        self.assertFalse(os.path.exists(os.path.dirname(self.path)))
        self.assertIsNone(status["saved"])
        self.assertFalse(status["dirty"])
        self.assertFalse(status["restart_required"])
        self.assertTrue(status["manual_restart_only"])
        self.assertEqual(status["running_launch"], DEFAULTS)

    def test_unknown_launch_is_not_replaced_by_defaults_as_evidence(self):
        self.assertIsNone(StartupStore(self.path).status()["running_launch"])

    def test_draft_is_not_saved_and_revision_prevents_old_requests(self):
        self.change("set", "recording", True)
        self.assertFalse(os.path.exists(self.path))
        self.assertFalse(self.store.launch_settings["recording"])
        with self.assertRaises(ValueError): self.store.change(0, "save")
        self.assertTrue(self.store.status()["dirty"])

    def test_save_load_and_launcher_use_exact_values_without_starting_anything(self):
        self.change("module", "steering_faults", True)
        self.change("set", "recorded_acceleration", True)
        self.change("save")
        self.assertTrue(self.store.status()["restart_required"])
        saved = read_file(self.path)[0]
        self.assertEqual(saved, self.store.draft)
        self.assertEqual(StartupStore(self.path, saved).status()["restart_required"], False)
        arguments = launch_arguments(self.path)
        self.assertIn("use_recorded_acceleration:=true", arguments)
        self.assertIn("use_recorded_steering:=false", arguments)
        self.assertIn("enable_steering_faults:=true", arguments)
        self.assertFalse(self.store.launch_settings["steering_proxy"])

    def test_invalid_routes_can_be_drafted_but_not_saved(self):
        for key in ("recorded_acceleration", "steering_faults"):
            self.change("set", key, True)
        self.assertEqual(len(self.store.status()["validation_errors"]), 2)
        with self.assertRaises(ValueError): self.change("save")
        self.assertFalse(os.path.exists(self.path))
        self.change("module", "all_faults", True)
        self.change("set", "goal_stop", True)
        with self.assertRaises(ValueError): self.change("save")

    def test_module_selection_preserves_non_module_preferences(self):
        self.change("set", "recording", True)
        self.change("set", "recorded_steering", True)
        self.change("module", "proxy_only", True)
        self.assertTrue(self.store.draft["recording"])
        self.assertTrue(self.store.draft["recorded_steering"])
        self.assertFalse(self.store.draft["steering_faults"])

    def test_discard_restores_saved_draft_without_changing_disk_or_running(self):
        self.change("save")
        with open(self.path, "rb") as stream: original = stream.read()
        self.change("set", "recording", True)
        self.change("discard")
        with open(self.path, "rb") as stream: self.assertEqual(stream.read(), original)
        self.assertEqual(self.store.draft, DEFAULTS)
        self.assertEqual(self.store.launch_settings, DEFAULTS)

    def test_restore_removes_only_owned_override_and_never_changes_running_settings(self):
        self.change("set", "recording", True)
        self.change("save")
        unrelated = os.path.join(os.path.dirname(self.path), "other.json")
        with open(unrelated, "w") as stream: stream.write("unrelated")
        self.change("restore")
        self.assertFalse(os.path.exists(self.path))
        self.assertTrue(os.path.exists(unrelated))
        self.assertEqual(self.store.draft, DEFAULTS)
        self.assertEqual(self.store.launch_settings, DEFAULTS)

    def test_external_change_or_other_session_save_is_not_overwritten_or_removed(self):
        other = StartupStore(self.path, DEFAULTS)
        self.change("save")
        with self.assertRaises(ValueError): other.change(0, "save")
        with self.assertRaises(ValueError): other.change(0, "restore")
        self.assertTrue(os.path.exists(self.path))
        self.assertTrue(other.status()["disk_error"])

    def test_malformed_unknown_and_invalid_files_do_not_fall_back_silently(self):
        os.makedirs(os.path.dirname(self.path))
        for data in ("{", "[]", json.dumps({"other": True}), json.dumps(dict(format=FORMAT, schema_version=1, settings={}))):
            with open(self.path, "w") as stream: stream.write(data)
            broken = StartupStore(self.path)
            self.assertTrue(broken.status()["disk_error"])
            with self.assertRaises(ValueError): broken.change(0, "save")
            with self.assertRaises(ValueError): launch_arguments(self.path)
            self.assertIn("enable_steering_proxy:=false", launch_arguments(self.path, True))

    def test_failed_atomic_replace_keeps_previous_file_and_cleans_temp(self):
        self.change("save")
        self.change("set", "recording", True)
        with mock.patch("startup_configuration.os.replace", side_effect=OSError("not writable")):
            with self.assertRaises(OSError): self.change("save")
        self.assertFalse(read_file(self.path)[0]["recording"])
        self.assertFalse(any(name.endswith(".tmp") for name in os.listdir(os.path.dirname(self.path))))

    def test_only_known_boolean_fields_are_allowed(self):
        for key, value in (("steering_proxy", "true"), ("unknown", True)):
            with self.assertRaises(ValueError): self.change("set", key, value)
        for name in MODULES:
            settings = dict(DEFAULTS)
            settings.update({key: True for key in MODULES[name]})
            validate(settings)

    def test_configured_launch_defaults_and_forwarding_match_schema(self):
        root = ET.parse(os.path.join(PACKAGE, "launch", "manager_real_world_configured.launch")).getroot()
        args = {item.get("name"): item.get("default") for item in root.findall("arg")}
        forwarded = {item.get("name"): item.get("value") for item in root.findall("include/arg")}
        for key, argument in ARGUMENTS.items():
            self.assertEqual(args[argument], str(DEFAULTS[key]).lower())
            if key not in ("lidar_path", "gnss_path"):
                self.assertEqual(forwarded[argument], "$(arg %s)" % argument)
        self.assertEqual(forwarded["enable_test_configuration_monitor"], "true")
        self.assertEqual(forwarded["enable_default_points_raw_relay"], "$(eval not arg('enable_lidar_path'))")
        sensors = root.find("node")
        self.assertEqual(sensors.get("if"), "$(eval arg('enable_lidar_path') or arg('enable_gnss_path'))")

    def test_services_are_session_and_revision_bound_and_never_publish_commands(self):
        import types
        services = {}
        class Response(object):
            def __init__(self, success, message): self.success, self.message = success, message
        srv = types.ModuleType("std_srvs.srv")
        srv.SetBool = srv.Trigger = object
        srv.SetBoolResponse = srv.TriggerResponse = Response
        def service(name, kind, callback):
            services[name] = callback
            return types.SimpleNamespace(shutdown=lambda reason: services.pop(name, None))
        with mock.patch.dict(sys.modules, {"std_srvs.srv": srv}):
            manager = StartupServices(types.SimpleNamespace(Service=service), "a" * 32, self.store)
        first = manager.status()
        old = services[first["service_root"] + "/save"]
        self.assertTrue(services[first["service_root"] + "/set/recording"](types.SimpleNamespace(data=True)).success)
        self.assertFalse(old(None).success)
        second = manager.status()
        self.assertNotEqual(first["service_root"], second["service_root"])
        self.assertNotIn(first["service_root"] + "/save", services)

    def test_partial_service_registration_is_cleaned_up_and_rejected_request_is_visible(self):
        import types
        services, fail = {}, [True]
        class Response(object):
            def __init__(self, success, message): self.success, self.message = success, message
        srv = types.ModuleType("std_srvs.srv")
        srv.SetBool = srv.Trigger = object
        srv.SetBoolResponse = srv.TriggerResponse = Response
        def service(name, kind, callback):
            if fail[0] and len(services) == 3: raise RuntimeError("registration failed")
            services[name] = callback
            return types.SimpleNamespace(shutdown=lambda reason: services.pop(name, None))
        with mock.patch.dict(sys.modules, {"std_srvs.srv": srv}):
            manager = StartupServices(types.SimpleNamespace(Service=service), "a" * 32, self.store)
        with self.assertRaises(RuntimeError): manager.status()
        self.assertFalse(services)
        self.assertIsNone(manager.registered_revision)
        fail[0] = False
        self.assertTrue(manager.status()["services_available"])
        self.assertFalse(manager.call(99, "save").success)
        self.assertFalse(manager.status()["last_request"]["success"])
        self.assertFalse(os.path.exists(self.path))


if __name__ == "__main__": unittest.main()
