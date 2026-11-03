"""Check configuration reports and launch settings using a simple ROS stand-in."""
import ast
import json
import os
import re
import sys
import types
import unittest
import xml.etree.ElementTree as ET

PACKAGE = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, os.path.join(PACKAGE, "src"))
# Avoid colliding with this test file's discovery name.
import importlib.util
spec = importlib.util.spec_from_file_location("configuration_monitor_logic", os.path.join(PACKAGE, "src", "test_configuration_monitor.py"))
monitor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(monitor)


class Obj(object):
    def __init__(self, **kwargs): self.__dict__.update(kwargs)


def graph():
    return ([ ["/steering_requested_phy", ["/steering_fault_proxy_live"]],
              ["/points_raw", ["/sensor_fault_injection_manager"]] ],
            [["/vehicle_commands", ["/manager"]]], [])


class ConfigurationMonitorTests(unittest.TestCase):
    def status(self, state=None, **kwargs):
        return monitor.make_status(monitor.get_registry(), "observer-a", "registry-hash", "base",
                                   {"steering_proxy": False}, state, now=100, **kwargs)

    def test_default_hints_and_registered_nodes_never_become_applied_state(self):
        result = self.status(graph())
        self.assertTrue(result["manager"]["registered"])
        self.assertFalse(result["baseline"]["restore_ready"])
        self.assertIsNone(result["manager"]["confirmed_boot_id"])
        self.assertEqual(result["graph"]["publishers"]["/steering_requested_phy"], ["/steering_fault_proxy_live"])
        for feature in result["features"]:
            self.assertEqual(feature["applied_state"], "unknown")
            self.assertIsNone(feature["applied_value"])
        steering = next(item for item in result["features"] if item["id"] == "steering_proxy")
        self.assertFalse(steering["launch_requested"])
        self.assertTrue(result["read_only"])
        self.assertFalse(any(result["actions"].values()))

    def test_graph_failure_unknown_not_off(self):
        result = self.status(graph_error="master offline")
        self.assertFalse(result["graph"]["available"])
        self.assertIsNone(result["manager"]["registered"])
        self.assertEqual(result["graph"]["error"], "master offline")
        self.assertTrue(all(item["applied_state"] == "unknown" for item in result["features"]))
        self.assertIsNone(next(item for item in result["features"] if item["id"] == "recording")["launch_requested"])

    def test_reports_are_not_configuration_acknowledgements(self):
        report = monitor.read_fault_report(json.dumps({"source": "sensor", "event": "configured", "fault": "LIDAR_RANGE_M_command", "value": 20, "active": False}))
        result = self.status(graph(), reports=[report])
        self.assertEqual(result["recent_fault_reports"][0]["event"], "configured")
        self.assertFalse(result["baseline"]["restore_ready"])
        self.assertTrue(all(item["applied_state"] == "unknown" for item in result["features"]))

    def test_invalid_reports_rejected(self):
        for value in ["{", "[]", "null", "{}", json.dumps({"source": "s", "fault": "f", "event": "active", "value": float("inf")}),
                      json.dumps({"source": "s", "fault": "f", "event": "active", "active": "true"})]:
            with self.assertRaises(ValueError): monitor.read_fault_report(value)

    def test_monitor_mocked_ros_wiring_is_read_only(self):
        pubs, subscriptions, shutdown = [], [], []
        rospy = types.ModuleType("rospy")
        rospy.get_param = lambda name, default=None: {"~profile": "all_faults", "~startup_request": {"steering_proxy": True}}.get(name, default)
        rospy.get_name = lambda: "/test_configuration_monitor"
        rospy.is_shutdown = lambda: False
        rospy.loginfo = lambda *args: None
        def publisher(topic, *args, **kwargs):
            pub = Obj(topic=topic, messages=[], latch=kwargs.get("latch"))
            pub.publish = pub.messages.append
            pubs.append(pub)
            return pub
        rospy.Publisher = publisher
        rospy.Subscriber = lambda topic, *args, **kwargs: subscriptions.append(topic)
        rospy.on_shutdown = lambda callback: (shutdown.append(callback), callback())
        master = Obj(getSystemState=graph, lookupNode=lambda name: "http://localhost:1234")
        rosgraph = types.ModuleType("rosgraph")
        rosgraph.Master = lambda name: master
        std = types.ModuleType("std_msgs.msg")
        std.String = Obj
        replacements = {"rospy": rospy, "rosgraph": rosgraph, "std_msgs.msg": std}
        saved = {key: sys.modules.get(key) for key in replacements}
        try:
            sys.modules.update(replacements)
            node = monitor.TestConfigurationMonitor()
            node.worker.join(1)
            self.assertFalse(node.worker.is_alive())
            self.assertEqual([pub.topic for pub in pubs], [monitor.REGISTRY_TOPIC, monitor.STATUS_TOPIC])
            self.assertTrue(all(pub.latch for pub in pubs))
            self.assertEqual(subscriptions, ["/adeye/physical_fault_status"])
            registry_message = json.loads(pubs[0].messages[0].data)
            self.assertEqual(registry_message["monitor_boot_id"], node.boot_id)
            for index in range(70):
                node.fault_callback(Obj(data=json.dumps({"source": "sensor", "event": "active", "fault": str(index)})))
            result = node.sample()
            self.assertEqual(len(result["recent_fault_reports"]), 64)
            self.assertEqual(result["profile"], "all_faults")
            self.assertEqual(result["manager"]["api_uri"], "http://localhost:1234")
            master.getSystemState = lambda: (_ for _ in ()).throw(RuntimeError("offline"))
            self.assertFalse(node.sample()["graph"]["available"])
        finally:
            for key, value in saved.items():
                if value is None: sys.modules.pop(key, None)
                else: sys.modules[key] = value

    def test_source_only_publishes_inventory_and_status_never_writes_parameters(self):
        with open(spec.origin) as source: tree = ast.parse(source.read())
        forbidden = {"Service", "ServiceProxy", "set_param", "delete_param", "kill_nodes", "Popen", "system", "launch"}
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                self.assertNotIn(node.func.attr, forbidden)
        publishers = [node.args[0].id for node in ast.walk(tree) if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute) and node.func.attr == "Publisher"]
        self.assertEqual(publishers, ["REGISTRY_TOPIC", "STATUS_TOPIC"])

    def test_opt_in_all_profiles_and_launch_argument_order(self):
        registry = monitor.get_registry()
        for name in ["manager_real_world.launch"] + [profile["launch_file"] for profile in registry["profiles"]]:
            root = ET.parse(os.path.join(PACKAGE, "launch", name)).getroot()
            args = {element.get("name"): element.get("default") for element in root.findall("arg")}
            self.assertEqual(args["enable_test_configuration_monitor"], "false", name)
            self.assertIn("test_configuration_profile", args)
            if name != "manager_real_world.launch":
                forwarded = {element.get("name"): element.get("value") for element in root.findall("include/arg")}
                self.assertEqual(forwarded["enable_test_configuration_monitor"], "$(arg enable_test_configuration_monitor)")
                self.assertEqual(forwarded["test_configuration_profile"], "$(arg test_configuration_profile)")
            else:
                defined = set()
                for element in root:
                    if element.tag == "arg": defined.add(element.get("name"))
                    if element.get("type") != "test_configuration_monitor.py": continue
                    self.assertEqual(element.get("if"), "$(arg enable_test_configuration_monitor)")
                    for child in element:
                        value = child.get("value", "")
                        for argument in re.findall(r"\$\(arg ([^)]+)\)|arg\('([^']+)'\)", value):
                            self.assertIn(argument[0] or argument[1], defined)

    def test_monitor_and_sibling_helper_installed(self):
        with open(os.path.join(PACKAGE, "CMakeLists.txt")) as source: cmake = source.read()
        self.assertIn("src/test_configuration_monitor.py", cmake)
        self.assertIn("install(FILES src/test_feature_registry.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})", cmake)


if __name__ == "__main__": unittest.main()
