"""Workflow simulation and launch-file evaluation, without ROS or car commands."""
import ast
import importlib.util
import itertools
import os
import sys
import tempfile
import threading
import types
import unittest
import xml.etree.ElementTree as ET

PACKAGE = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(PACKAGE, "src"))
from startup_configuration import ARGUMENTS, DEFAULTS, MODULES, StartupStore
from configuration_diagnostics import diagnose, expected_routes
from launch_configured_manager import launch_arguments
from test_feature_registry import get_registry

spec = importlib.util.spec_from_file_location("workflow_monitor", os.path.join(PACKAGE, "src", "test_configuration_monitor.py"))
monitor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(monitor)


def expand(text, args):
    if text is None: return None
    if text.startswith("$(arg "): return args[text[6:-1]]
    if text.startswith("$(eval "):
        return eval(text[7:-1], {"__builtins__": {}}, {"arg": lambda name: args[name]})
    return {"true": True, "false": False}.get(text, text)


def launched_nodes(settings):
    """Evaluate this pair of launch files only; not a replacement for roslaunch."""
    nodes = {}
    def visit(filename, supplied):
        root = ET.parse(os.path.join(PACKAGE, "launch", filename)).getroot()
        args = {}
        for item in root.findall("arg"):
            name = item.get("name")
            args[name] = supplied[name] if name in supplied else expand(item.get("default"), args)
        for item in root:
            if item.get("if") and not expand(item.get("if"), args): continue
            if item.get("unless") and expand(item.get("unless"), args): continue
            if item.tag == "include" and item.get("file") == "$(find adeye)/launch/manager_real_world.launch":
                visit("manager_real_world.launch", {child.get("name"): expand(child.get("value"), args) for child in item.findall("arg")})
            elif item.tag == "node":
                name = "/" + item.get("name")
                if name in nodes: raise AssertionError("Duplicate node " + name)
                nodes[name] = {child.get("name"): expand(child.get("value"), args) for child in item.findall("param")}
    visit("manager_real_world_configured.launch", {ARGUMENTS[key]: value for key, value in settings.items()})
    return nodes


def registered_graph(settings):
    pubs, subs = {}, {}
    for route in expected_routes(settings):
        pubs.setdefault(route["topic"], set()).add(route["publisher"] or "/source")
        if route["subscriber"]: subs.setdefault(route["topic"], set()).add(route["subscriber"])
    return ([ [topic, list(nodes)] for topic, nodes in pubs.items() ], [[topic, list(nodes)] for topic, nodes in subs.items()], [])


class WorkflowTests(unittest.TestCase):
    def test_every_module_and_live_replay_combination_matches_launch_wiring(self):
        for module, replay in itertools.product(MODULES, itertools.product((False, True), repeat=2)):
            settings = dict(DEFAULTS)
            settings.update({key: True for key in MODULES[module]})
            for axis, recorded in zip(("steering", "acceleration"), replay):
                settings["recorded_" + axis] = recorded
                if recorded: settings[axis + "_proxy"] = True
            with self.subTest(module=module, replay=replay):
                nodes = launched_nodes(settings)
                for route in expected_routes(settings):
                    if route["publisher"]: self.assertIn(route["publisher"], nodes)
                    if route["subscriber"] and route["id"].endswith(".input"):
                        self.assertIn(route["subscriber"], nodes)
                        parameter = "input_topic" if route["id"].startswith(("steering", "acceleration")) else route["id"].split(".")[0] + "_input_topic"
                        self.assertEqual(nodes[route["subscriber"]][parameter], route["topic"])
                self.assertEqual("/points_raw_relay" in nodes, not settings["lidar_path"])
                for axis, producer in (("steering", "/ctrl_cmd_republisher"), ("acceleration", "/vehicle_controller")):
                    if not settings["recorded_" + axis]:
                        parameter = "steering_output_topic" if axis == "steering" else "topic_acceleration_request"
                        expected = ("/nominal/" if settings[axis + "_proxy"] else "/") + axis + "_requested_phy"
                        self.assertEqual(nodes[producer][parameter], expected)
                    else: self.assertNotIn(producer, nodes)
                snapshot = nodes["/test_configuration_monitor"]
                for key in settings: self.assertEqual(snapshot["startup_settings/" + key], settings[key])

    def test_save_restart_restore_and_ignore_saved_workflow(self):
        with tempfile.TemporaryDirectory() as directory:
            path = os.path.join(directory, "startup.json")
            before = StartupStore(path, DEFAULTS)
            before.change(0, "module", "all_faults", True)
            before.change(1, "set", "recorded_acceleration", True)
            before.change(2, "save")
            self.assertEqual(before.launch_settings, DEFAULTS)
            command = launch_arguments(path)
            self.assertIn("use_recorded_acceleration:=true", command)
            after = StartupStore(path, before.saved)
            self.assertFalse(after.status()["restart_required"])
            self.assertFalse(after.status()["dirty"])
            self.assertEqual(after.revision, 0)
            self.assertFalse(any(check["state"] == "warning" for check in diagnose(registered_graph(after.launch_settings), after.status())["checks"]))
            after.change(0, "restore")
            self.assertTrue(after.launch_settings["recorded_acceleration"])
            self.assertTrue(after.status()["restart_required"])
            self.assertIn("use_recorded_acceleration:=false", launch_arguments(path, True))

    def test_missing_replay_publisher_reports_information_only(self):
        settings = dict(DEFAULTS, steering_proxy=True, recorded_steering=True)
        graph = registered_graph(settings)
        graph[0][:] = [pair for pair in graph[0] if pair[0] != "/replay/steering_requested_phy"]
        result = diagnose(graph, {"running_launch": settings, "disk_error": "", "validation_errors": [], "restart_required": False})
        self.assertTrue(result["information_only"])
        self.assertTrue(any(check["id"] == "steering.input" and check["state"] == "warning" for check in result["checks"]))
        self.assertTrue(settings["recorded_steering"])

    def test_competing_output_and_missing_proxy_connection_are_reported(self):
        settings = dict(DEFAULTS, acceleration_proxy=True)
        graph = registered_graph(settings)
        graph[0].append(["/acceleration_requested_phy", ["/intruder", "/acceleration_fault_proxy_live"]])
        graph[1][:] = []
        checks = diagnose(graph, {"running_launch": settings, "disk_error": "", "validation_errors": [], "restart_required": False})["checks"]
        self.assertTrue(any("multiple publishers" in item["detail"] for item in checks))
        self.assertTrue(any("expected subscriber" in item["detail"] for item in checks))

    def test_unknown_graph_and_launch_never_report_ready(self):
        for state, startup in ((None, None), (([], [], []), None)):
            result = diagnose(state, startup)
            self.assertTrue(all(item["state"] == "unknown" for item in result["checks"]))

    def test_diagnostics_do_not_read_or_write_control_channels(self):
        with open(os.path.join(PACKAGE, "src", "configuration_diagnostics.py")) as stream: tree = ast.parse(stream.read())
        forbidden = {"publish", "set_param", "delete_param", "ServiceProxy", "Popen", "execvp", "system", "sleep"}
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute): self.assertNotIn(node.func.attr, forbidden)

    def monitor_fixture(self):
        node = monitor.TestConfigurationMonitor.__new__(monitor.TestConfigurationMonitor)
        node.registry, node.boot_id, node.registry_id, node.profile = get_registry(), "a" * 32, "registry", "configured"
        node.startup_request, node.run_id = {}, "run"
        node.master = types.SimpleNamespace(getSystemState=lambda: ([], [], []))
        node.clock = lambda: 1
        node.lock, node.reports = threading.Lock(), {}
        node.control_errors, node.last_availability, node.startup = {}, None, None
        policy = dict(enabled=True, session_id=node.boot_id, commands={}, groups={}, owned_override_count=0,
            reservation_count=0, pending_panel_count=0, scope="availability", persistence="session_only")
        node.availability = types.SimpleNamespace(status=lambda: policy)
        return node

    def test_startup_service_failure_does_not_hide_runtime_status(self):
        node = self.monitor_fixture()
        with tempfile.TemporaryDirectory() as directory:
            store = StartupStore(os.path.join(directory, "startup.json"), DEFAULTS)
            def fail(): raise RuntimeError("service registration failed")
            node.startup = types.SimpleNamespace(status=fail, store=store)
            result = node.sample()
            self.assertIn("availability", result)
            self.assertFalse(result["startup"]["services_available"])
            self.assertFalse(result["actions"]["save_available"])
            self.assertTrue(any("service registration failed" in item["detail"] for item in result["diagnostics"]["checks"]))
            self.assertEqual(store.launch_settings, DEFAULTS)

    def test_availability_failure_retains_policy_without_new_applied_claims(self):
        node = self.monitor_fixture()
        previous = node.sample()["availability"]
        def fail(): raise RuntimeError("availability failed")
        node.availability.status = fail
        result = node.sample()
        self.assertEqual(result["availability"]["commands"], previous["commands"])
        self.assertTrue(result["availability"]["read_error"])
        self.assertTrue(all(item["applied_state"] == "unknown" for item in result["features"]))
        self.assertFalse(result["actions"]["apply_available"])

    def test_missing_component_does_not_look_applied_from_a_stored_acknowledgement(self):
        node = self.monitor_fixture()
        policy = node.availability.status()
        policy["commands"]["STEEROFFSET_command"] = dict(enabled=True, component="/gone", boot_id="old", revision=1)
        result = node.sample()
        feature = next(item for item in result["features"] if item["id"] == "fault.STEEROFFSET")
        self.assertEqual(feature["applied_state"], "unknown")
        self.assertIsNone(feature["applied_value"])
        self.assertTrue(result["availability"]["commands"]["STEEROFFSET_command"]["enabled"])
        self.assertTrue(any("not listed in the graph" in item["detail"] for item in result["diagnostics"]["checks"]))

    def test_unexpected_observer_failure_keeps_reporting_without_retrying_commands(self):
        node = self.monitor_fixture()
        count, reports = [0], []
        sample = node.sample
        def broken_sample():
            if count[0] == 0: return sample()
            raise RuntimeError("unexpected observer failure")
        node.sample = broken_sample
        node.rate = 1
        node.shutdown = types.SimpleNamespace(is_set=lambda: False, wait=lambda seconds: count.__setitem__(0, count[0] + 1))
        node.rospy = types.SimpleNamespace(is_shutdown=lambda: count[0] >= 2)
        node.String = lambda **kwargs: types.SimpleNamespace(**kwargs)
        node.pub = types.SimpleNamespace(publish=reports.append)
        node.run()
        self.assertEqual(len(reports), 2)
        import json
        failure = json.loads(reports[-1].data)
        self.assertEqual(failure["sequence"], 2)
        self.assertTrue(all(item["applied_state"] == "unknown" for item in failure["features"]))
        self.assertFalse(any(failure["actions"].values()))
        self.assertTrue(failure["diagnostics"]["information_only"])
        self.assertIn("unexpected observer failure", failure["availability"]["read_error"])


if __name__ == "__main__": unittest.main()
