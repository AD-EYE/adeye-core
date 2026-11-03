"""Test fault switches using the real sensor/configuration code and a simple ROS stand-in."""
import json
import os
import sys
import threading
import types
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
import test_lidar_timestamp_status as lidar_fixture
from test_lidar_timestamp_status import Obj
from fault_availability import SensorAvailability
from fault_availability_coordinator import AvailabilityCoordinator, read_state
from test_feature_registry import get_registry


class AvailabilityTests(unittest.TestCase):
    def setUp(self):
        fixture = lidar_fixture.LidarTimestampTests()
        fixture.setUp()
        self.fixture, self.node = fixture, fixture.node
        rospy = self.node.command_callback.__globals__["rospy"]
        rospy.get_name = lambda: "/sensor_fault_injection_manager"
        self.services, self.control_requests = {}, []
        coordinator = [None]
        class Publisher(object):
            def __init__(pub, topic): pub.topic = topic
            def publish(pub, message):
                if pub.topic == "/adeye/fault_availability_state" and coordinator[0]: coordinator[0].state_callback(message)
                if pub.topic == "/adeye/fault_availability_control":
                    self.control_requests.append(json.loads(message.data))
                    self.gate.control(message)
        rospy.Publisher = lambda topic, *args, **kwargs: Publisher(topic)
        def service(name, kind, callback):
            self.services[name] = callback
            return Obj(shutdown=lambda *args: self.services.pop(name, None))
        rospy.Service = service
        std = types.ModuleType("std_msgs.msg"); std.String = Obj
        srv = types.ModuleType("std_srvs.srv")
        srv.SetBool = srv.Trigger = Obj
        srv.SetBoolResponse = srv.TriggerResponse = lambda success, message: Obj(success=success, message=message)
        replacements = {"rospy": rospy, "std_msgs.msg": std, "std_srvs.srv": srv}
        saved = {key: sys.modules.get(key) for key in replacements}
        try:
            sys.modules.update(replacements)
            self.gate = SensorAvailability(self.node, ["LIDAR_RANGE_M_command", "LIDAR_NOISE_STDDEV_M_command"])
            self.node.availability = self.gate
            self.coordinator = AvailabilityCoordinator(rospy, Obj, get_registry(), "a" * 32)
            coordinator[0] = self.coordinator
            self.gate.publish()
        finally:
            for key, value in saved.items():
                if value is None: sys.modules.pop(key, None)
                else: sys.modules[key] = value

    def set(self, enabled): return self.coordinator.set_service("LIDAR_RANGE_M_command", enabled)

    def test_old_boot_cannot_replace_new_controller_confirmation(self):
        old = self.coordinator.resolve("LIDAR_RANGE_M_command")
        newer = dict(old, boot_id="new-boot")
        self.coordinator.state_callback(Obj(data=json.dumps(newer)))
        self.coordinator.state_callback(Obj(data=json.dumps(old)))
        self.assertEqual(self.coordinator.resolve("LIDAR_RANGE_M_command")["boot_id"], "new-boot")

    def test_missing_and_ambiguous_switches_have_different_explanations(self):
        self.assertIn("No controller", self.coordinator.status()["unconfirmed"]["STEEROFFSET_command"])
        state = self.coordinator.resolve("LIDAR_RANGE_M_command")
        self.coordinator.state_callback(Obj(data=json.dumps(dict(state, component="/duplicate"))))
        self.assertIn("Multiple controllers", self.coordinator.status()["unconfirmed"]["LIDAR_RANGE_M_command"])

    def test_group_off_blocks_all_children_but_preserves_relay_and_individual_settings(self):
        self.assertTrue(self.set(False).success)
        self.assertTrue(self.coordinator.set_service("LIDAR_GROUP_command", False).success)
        self.assertEqual(self.gate.blocked_activation("LIDAR_NOISE_STDDEV_M_command=0.05"), "LIDAR_NOISE_STDDEV_M_command")
        self.assertEqual(self.gate.blocked_activation("LIDAR_NOISE_STDDEV_M_command=0"), "")
        self.node.lidar_callback(self.fixture.cloud())
        self.assertTrue(self.node.lidar_pub.messages)
        status = self.coordinator.status()
        self.assertFalse(status["groups"]["lidar_fault_processing"]["enabled"])
        self.assertTrue(status["commands"]["LIDAR_NOISE_STDDEV_M_command"]["enabled"])
        self.assertFalse(status["commands"]["LIDAR_NOISE_STDDEV_M_command"]["effective_enabled"])
        self.assertFalse(self.coordinator.reserve_service("LIDAR_NOISE_STDDEV_M_command").success)
        self.assertTrue(self.coordinator.set_service("LIDAR_GROUP_command", True).success)
        self.assertFalse(self.gate.entries["LIDAR_RANGE_M_command"]["enabled"])
        self.assertEqual(self.gate.blocked_activation("LIDAR_NOISE_STDDEV_M_command=0.05"), "")

    def test_group_cannot_change_during_reservation_pending_test_or_busy_controller(self):
        self.assertTrue(self.coordinator.reserve_service("LIDAR_RANGE_M_command").success)
        self.assertFalse(self.coordinator.set_service("LIDAR_GROUP_command", False).success)
        token = self.coordinator.reservations["LIDAR_RANGE_M_command"]
        self.coordinator.release("LIDAR_RANGE_M_command", token)
        self.coordinator.pending_gui["panel"] = "LIDAR_NOISE_STDDEV_M_command"
        self.assertFalse(self.coordinator.set_service("LIDAR_GROUP_command", False).success)
        self.coordinator.pending_gui.clear()
        self.fixture.command("LIDAR_RANGE_M_command=5")
        self.assertFalse(self.coordinator.set_service("LIDAR_GROUP_command", False).success)
        self.assertEqual(self.node.active_fault, "LIDAR_RANGE_M_command")

    def test_reset_restores_actual_group_baseline_without_starting_fault(self):
        parent = self.gate.entries["LIDAR_GROUP_command"]
        parent.update(baseline=False, enabled=False)
        self.gate.publish()
        self.assertTrue(self.coordinator.set_service("LIDAR_GROUP_command", True).success)
        self.assertTrue(self.set(False).success)
        self.assertEqual(self.coordinator.status()["owned_override_count"], 2)
        self.assertTrue(self.coordinator.reset_service(None).success)
        self.assertEqual(self.control_requests[-2]["fault"], "LIDAR_GROUP_command")
        self.assertFalse(parent["enabled"])
        self.assertTrue(self.gate.entries["LIDAR_RANGE_M_command"]["enabled"])
        self.assertEqual(self.node.active_fault, "")

    def test_managed_proxy_source_keeps_startup_off_group_available_for_runtime_enable(self):
        root = os.path.join(os.path.dirname(__file__), "..", "src")
        for name, group in (("steering_fault_proxy.cpp", "STEER"), ("recorded_acceleration_fault_proxy.cpp", "ACCEL")):
            with open(os.path.join(root, name)) as source:
                text = source.read()
            self.assertIn("if (enable_fault_injection_ || availability_enabled)", text)
            self.assertIn('"%s_GROUP_command", enable_fault_injection_' % group, text)
            self.assertIn("if (availability_enabled) availability_", text)

    def test_disable_blocks_direct_activation_but_keeps_relay_and_stop(self):
        self.assertTrue(self.set(False).success)
        self.fixture.command("LIDAR_RANGE_M_command=5")
        self.assertEqual(self.node.active_fault, "")
        self.assertEqual(json.loads(self.node.status_pub.messages[-1].data)["event"], "rejected")
        self.node.lidar_callback(self.fixture.cloud())
        self.assertTrue(self.node.lidar_pub.messages)
        self.assertEqual(self.gate.blocked_activation("LIDAR_RANGE_M_command=0"), "")
        self.assertEqual(self.gate.blocked_activation("LIDAR_NOISE_STDDEV_M_command=0.05"), "")

    def test_busy_change_rejected_without_cancelling_fault(self):
        self.fixture.command("LIDAR_RANGE_M_command=5")
        self.gate.publish()
        result = self.set(False)
        self.assertFalse(result.success)
        self.assertEqual(self.node.active_fault, "LIDAR_RANGE_M_command")
        self.assertTrue(self.gate.entries["LIDAR_RANGE_M_command"]["enabled"])

    def test_idle_snapshot_does_not_bypass_live_busy_check(self):
        self.fixture.command("LIDAR_RANGE_M_command=5")  # Do not refresh coordinator snapshot.
        self.assertFalse(self.set(False).success)
        self.assertTrue(self.gate.entries["LIDAR_RANGE_M_command"]["enabled"])

    def test_reset_only_owned_override_and_no_change_no_requests(self):
        self.assertTrue(self.coordinator.reset_service(None).success)
        self.assertEqual(self.control_requests, [])
        self.assertTrue(self.set(False).success)
        self.assertEqual(self.coordinator.status()["owned_override_count"], 1)
        self.assertTrue(self.coordinator.reset_service(None).success)
        self.assertTrue(self.gate.entries["LIDAR_RANGE_M_command"]["enabled"])
        self.assertEqual(self.node.active_fault, "")
        self.assertEqual(self.coordinator.status()["owned_override_count"], 0)
        before = len(self.control_requests)
        self.coordinator.reset_service(None)
        self.assertEqual(before, len(self.control_requests))

    def test_false_component_baseline_is_restored_not_universal_true(self):
        entry = self.gate.entries["LIDAR_RANGE_M_command"]
        entry.update(baseline=False, enabled=False)
        self.gate.publish()
        self.assertTrue(self.set(True).success)
        self.assertTrue(self.coordinator.reset_service(None).success)
        self.assertFalse(entry["enabled"])

    def test_other_session_override_not_overwritten(self):
        self.assertTrue(self.set(False).success)
        self.coordinator.owner = "b" * 32
        self.assertFalse(self.set(True).success)
        self.assertTrue(self.coordinator.reset_service(None).success)
        self.assertFalse(self.gate.entries["LIDAR_RANGE_M_command"]["enabled"])

    def test_pending_reservation_blocks_changes_until_explicit_release(self):
        reservation = self.coordinator.reserve_service("LIDAR_RANGE_M_command")
        self.assertTrue(reservation.success)
        self.assertFalse(self.set(False).success)
        self.assertFalse(self.coordinator.reserve_service("LIDAR_RANGE_M_command").success)
        self.assertTrue(self.services[reservation.message](None).success)
        self.assertTrue(self.set(False).success)

    def test_legacy_pending_gui_and_disabled_reservation_rejected(self):
        self.coordinator.pending_callback(Obj(data=json.dumps(dict(component="gui-a", pending=True, fault="LIDAR_RANGE_M_command"))))
        self.assertFalse(self.set(False).success)
        self.coordinator.pending_callback(Obj(data=json.dumps(dict(component="gui-a", pending=False))))
        self.assertTrue(self.set(False).success)
        self.assertFalse(self.coordinator.reserve_service("LIDAR_RANGE_M_command").success)

    def test_closed_panel_reservation_released_only_after_confirmed_fault_end(self):
        self.assertTrue(self.coordinator.reserve_service("LIDAR_RANGE_M_command").success)
        self.coordinator.fault_report(dict(fault="LIDAR_RANGE_M_command", event="rejected", active=False))
        self.assertEqual(self.coordinator.status()["reservation_count"], 1)
        self.coordinator.fault_report(dict(fault="LIDAR_RANGE_M_command", event="active", active=True))
        self.coordinator.fault_report(dict(fault="LIDAR_RANGE_M_command", event="timeout", active=False))
        self.assertEqual(self.coordinator.status()["reservation_count"], 0)

    def test_revisions_and_boot_mismatch_rejected_by_component(self):
        self.assertTrue(self.set(False).success)
        old = dict(self.control_requests[0])
        old.update(request_id="c" * 32, value=True)
        self.gate.control(Obj(data=json.dumps(old)))
        self.assertFalse(self.gate.entries["LIDAR_RANGE_M_command"]["enabled"])
        old.update(revision=1, boot_id="old-boot")
        self.gate.control(Obj(data=json.dumps(old)))
        self.assertFalse(self.gate.entries["LIDAR_RANGE_M_command"]["enabled"])

    def test_ambiguous_components_and_bad_states_not_assumed_enabled(self):
        state = self.coordinator.resolve("LIDAR_RANGE_M_command")
        duplicate = dict(state, component="/another_sensor")
        self.coordinator.state_callback(Obj(data=json.dumps(duplicate)))
        self.assertFalse(self.set(False).success)
        self.assertNotIn("LIDAR_RANGE_M_command", self.coordinator.status()["commands"])
        for value in ["{}", "[]", "null"]:
            with self.assertRaises(ValueError): read_state(value, self.coordinator.commands)

    def test_proxy_source_guards_cover_all_continuous_commands_and_never_change_can(self):
        package = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        for filename, prefix in [("steering_fault_proxy.cpp", "STEER"), ("recorded_acceleration_fault_proxy.cpp", "ACCEL")]:
            with open(os.path.join(package, "src", filename)) as source: text = source.read()
            self.assertIn('"' + prefix + 'CONSTANT_command"', text)
            self.assertIn("availability_->blockedActivation(message->data)", text)
            guard = text.index("availability_->blockedActivation(message->data)")
            callback = text.index("void commandCallback")
            self.assertGreater(guard, callback)
        with open(os.path.join(package, "include", "fault_availability.h")) as source: header = source.read()
        self.assertIn('command.find("CONSTANT_command")', header)
        self.assertIn("component restarted or availability revision changed", header)
        self.assertNotIn("startVIM", header)
        self.assertNotIn("ACCELERATE_command", header)
        with open(os.path.join(package, "src", "fault_injections", "SensorFaultInjectionManager.py")) as source: sensor = source.read()
        self.assertLess(sensor.index("self.availability = SensorAvailability("), sensor.index("self.command_sub = rospy.Subscriber("))

    def test_missing_acknowledgement_is_unknown_and_never_retried(self):
        requests = []
        self.coordinator.pub.publish = lambda message: requests.append(message)
        clock = [0]
        def now():
            clock[0] += 1
            return clock[0]
        self.coordinator.clock = now
        result = self.set(False)
        self.assertFalse(result.success)
        self.assertIn("outcome unknown", result.message)
        self.assertEqual(len(requests), 1)
        self.assertTrue(self.gate.entries["LIDAR_RANGE_M_command"]["enabled"])

    def test_release_is_idempotent_after_backend_terminal_report(self):
        reservation = self.coordinator.reserve_service("LIDAR_RANGE_M_command")
        self.coordinator.fault_report(dict(fault="LIDAR_RANGE_M_command", event="active", active=True))
        self.coordinator.fault_report(dict(fault="LIDAR_RANGE_M_command", event="reset", active=False))
        self.assertTrue(self.services[reservation.message](None).success)
        self.assertEqual(self.coordinator.status()["reservation_count"], 0)

    def test_ambiguous_owned_override_remains_visible_as_owned_not_reset_complete(self):
        self.assertTrue(self.set(False).success)
        state = self.coordinator.resolve("LIDAR_RANGE_M_command")
        self.coordinator.state_callback(Obj(data=json.dumps(dict(state, component="/duplicate"))))
        self.assertGreater(self.coordinator.status()["owned_override_count"], 0)
        self.assertFalse(self.coordinator.reset_service(None).success)

    def test_partial_reset_does_not_claim_success_or_restore_commands(self):
        self.assertTrue(self.set(False).success)
        self.assertTrue(self.coordinator.set_service("LIDAR_NOISE_STDDEV_M_command", False).success)
        original = self.coordinator.pub.publish
        def publish(message):
            original(message)
            if json.loads(message.data)["operation"] == "reset": self.node.recovery_hold = True
        self.coordinator.pub.publish = publish
        result = self.coordinator.reset_service(None)
        self.assertFalse(result.success)
        self.assertIn("incomplete", result.message)
        self.assertEqual(self.node.active_fault, "")
        self.assertTrue(self.node.recovery_hold)


if __name__ == "__main__": unittest.main()
