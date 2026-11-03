"""Check manual-reset information without a car or CAN connection."""
import ast
import json
import os
import re
import sys
import threading
import types
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
import vehicle_recovery_monitor as recovery


class Obj(object):
    def __init__(self, **values):
        self.__dict__.update(values)


def snapshot(**changes):
    result = {
        "schema_version": 1, "source": "ros2can", "state": "FAULT",
        "reason": "DRIVER_BRAKE_INTERVENTION", "detail": "Driver pressed brake",
        "fault_entry_reason": "DRIVER_BRAKE_INTERVENTION", "fault_entry_detail": "Driver pressed brake",
        "speed_mps": 0.0, "brake_interrupt": 0, "accelerator_interrupt": 0,
        "steering_interrupt": 0, "ad_activation_ok": 1, "ad_mode_state": 0,
        "wall_time_s": 100, "can_sample_freshness_verified": False,
    }
    result.update(changes)
    return result


def assess(**changes):
    return recovery.assess_recovery(snapshot(**changes), input_age_s=0, now=100)


class RecoveryPolicyTests(unittest.TestCase):
    def assert_read_only(self, result):
        self.assertTrue(result["assessment_only"])
        for field in ["recovery_actions_implemented", "recovery_enabled",
                      "recovery_command_available", "eligible_for_recovery"]:
            self.assertFalse(result[field])
        self.assertIsNone(result["vehicle_fault_confirmed"])

    def test_driver_brake_candidate_never_becomes_ready_or_eligible(self):
        result = assess()
        self.assertEqual(result["assessment_state"], "CANDIDATE_UNVERIFIED")
        self.assertTrue(result["software_fault_latched"])
        self.assertTrue(result["candidate_for_future_recovery"])
        self.assertTrue(result["observed_prerequisites_met"])
        self.assertIn("VEHICLE_RESET_SEQUENCE_UNVERIFIED", result["blockers"])
        self.assertIn("CAN_SAMPLE_FRESHNESS_UNVERIFIED", result["blockers"])
        self.assert_read_only(result)

    def test_accelerator_candidate_has_the_same_read_only_policy(self):
        result = assess(reason="DRIVER_ACCELERATOR_INTERVENTION",
                        fault_entry_reason="DRIVER_ACCELERATOR_INTERVENTION")
        self.assertTrue(result["candidate_for_future_recovery"])
        self.assert_read_only(result)

    def test_unsupported_fault_classes_are_blocked(self):
        for reason, category in [("CAN_SIGNAL_INVALID", "CAN_COMMUNICATION_FAULT"),
                                 ("WHEEL_LOCK_FAILED", "WHEEL_LOCK_FAULT"),
                                 ("UNKNOWN_CAR_DISENGAGEMENT", "UNSUPPORTED_OR_UNKNOWN_FAULT"),
                                 ("STOP_COMMAND", "UNSUPPORTED_OR_UNKNOWN_FAULT"),
                                 ("DRIVER_STEERING_INTERVENTION", "UNSUPPORTED_OR_UNKNOWN_FAULT")]:
            result = assess(reason=reason, fault_entry_reason=reason)
            self.assertEqual(result["assessment_state"], "BLOCKED")
            self.assertEqual(result["fault_category"], category)
            self.assertFalse(result["candidate_for_future_recovery"])
            self.assertIn("FAULT_REASON_NOT_SUPPORTED_FOR_RECOVERY", result["blockers"])
            self.assert_read_only(result)

    def test_first_driver_cause_is_retained_but_later_can_fault_blocks_candidate(self):
        result = assess(reason="CAN_SIGNAL_INVALID")
        self.assertEqual(result["fault_entry_category"], "DRIVER_INTERVENTION_CANDIDATE")
        self.assertEqual(result["fault_category"], "CAN_COMMUNICATION_FAULT")
        self.assertFalse(result["candidate_for_future_recovery"])

    def test_unknown_initial_cause_cannot_be_overridden_by_driver_reason(self):
        result = assess(fault_entry_reason="UNKNOWN_CAR_DISENGAGEMENT")
        self.assertFalse(result["candidate_for_future_recovery"])

    def test_non_fault_states_do_not_request_recovery(self):
        for state in ["INITIALIZING", "ENABLED", "ENGAGED"]:
            result = assess(state=state)
            self.assertEqual(result["assessment_state"], "NO_SOFTWARE_FAULT")
            self.assertFalse(result["software_fault_latched"])
            self.assertFalse(result["candidate_for_future_recovery"])
            self.assert_read_only(result)

    def test_moving_in_either_direction_blocks_observed_conditions(self):
        for speed in [-2, 2]:
            result = assess(speed_mps=speed)
            self.assertEqual(result["assessment_state"], "BLOCKED")
            self.assertFalse(result["observed_conditions"]["stationary_from_cached_speed"])

    def test_active_ad_and_each_intervention_block_observed_conditions(self):
        for changed in [{"ad_mode_state": 1}, {"brake_interrupt": 1},
                        {"accelerator_interrupt": 1}, {"steering_interrupt": 1},
                        {"ad_activation_ok": 0}]:
            result = assess(**changed)
            self.assertEqual(result["assessment_state"], "BLOCKED")
            self.assertFalse(result["observed_prerequisites_met"])
            self.assert_read_only(result)

    def test_missing_speed_is_not_standstill_and_other_missing_values_are_unknown(self):
        for field in ["speed_mps", "brake_interrupt", "accelerator_interrupt",
                      "steering_interrupt", "ad_mode_state", "ad_activation_ok"]:
            result = assess(**{field: None})
            self.assertEqual(result["assessment_state"], "BLOCKED")
            self.assertTrue(any(reason.endswith("_UNKNOWN") for reason in result["blockers"]))

    def test_unrecognized_can_encodings_are_unknown_not_clear(self):
        for field in ["brake_interrupt", "accelerator_interrupt", "steering_interrupt",
                      "ad_mode_state", "ad_activation_ok"]:
            result = assess(**{field: 3})
            self.assertEqual(result["assessment_state"], "BLOCKED")

    def test_freshness_warnings_do_not_change_assessment_or_add_action(self):
        old = recovery.assess_recovery(snapshot(), input_age_s=20, now=120)
        current = assess()
        self.assertEqual(old["assessment_state"], current["assessment_state"])
        self.assertEqual(old["blockers"], current["blockers"])
        self.assertTrue(old["freshness_warnings"])
        self.assertTrue(old["freshness_checks_informational_only"])
        self.assert_read_only(old)

    def test_cannot_claim_verified_can_freshness_from_input_flag(self):
        result = assess(can_sample_freshness_verified=True)
        self.assertFalse(result["can_sample_freshness_verified"])
        self.assertIn("CAN_SAMPLE_FRESHNESS_UNVERIFIED", result["blockers"])

    def test_no_input_reports_waiting_not_zero_speed(self):
        result = recovery.assess_recovery(None)
        self.assertEqual(result["assessment_state"], "WAITING_FOR_INPUT")
        self.assertIsNone(result["observed_prerequisites_met"])
        self.assertEqual(result["observations"], {})
        self.assert_read_only(result)

    def test_snapshot_validation_rejects_incomplete_and_nonfinite_inputs(self):
        valid = recovery.read_snapshot(json.dumps(snapshot()))
        self.assertEqual(valid["speed_mps"], 0)
        for value in [None, [], {}, snapshot(schema_version=2), snapshot(source="other"),
                      snapshot(state="READY"), snapshot(speed_mps=float("nan")),
                      snapshot(speed_mps="0"), snapshot(speed_mps=True), snapshot(wall_time_s=float("inf"))]:
            with self.assertRaises(ValueError):
                recovery.read_snapshot(json.dumps(value))


class RecoveryAdapterTests(unittest.TestCase):
    def construct(self, params=None):
        publishers, subscribers, services = [], [], []
        rospy = types.ModuleType("rospy")
        rospy.get_param = lambda name, default=None: (params or {}).get(name, default)
        rospy.Publisher = lambda topic, *args, **kwargs: publishers.append((topic, kwargs)) or Obj()
        rospy.Subscriber = lambda topic, *args, **kwargs: subscribers.append(topic) or Obj()
        rospy.Service = lambda *args, **kwargs: services.append(args)
        rospy.on_shutdown = lambda callback: None
        rospy.loginfo = lambda *args: None
        std = types.ModuleType("std_msgs")
        std.msg = types.ModuleType("std_msgs.msg")
        std.msg.String = Obj
        replacements = {"rospy": rospy, "std_msgs": std, "std_msgs.msg": std.msg}
        saved = {key: sys.modules.get(key) for key in replacements}
        thread = recovery.threading.Thread
        try:
            sys.modules.update(replacements)
            recovery.threading.Thread = lambda **kwargs: Obj(start=lambda: None)
            node = recovery.VehicleRecoveryMonitor()
            return node, publishers, subscribers, services
        finally:
            recovery.threading.Thread = thread
            for key, value in saved.items():
                if value is None:
                    sys.modules.pop(key, None)
                else:
                    sys.modules[key] = value

    def test_only_status_publisher_and_observation_subscriber_no_service(self):
        node, publishers, subscribers, services = self.construct()
        self.assertEqual(publishers, [("/adeye/vehicle_recovery_status", {"queue_size": 1, "latch": True})])
        self.assertEqual(subscribers, ["/adeye/vehicle_recovery_inputs"])
        self.assertEqual(services, [])
        self.assertEqual(node.status()["assessment_state"], "WAITING_FOR_INPUT")

    def test_bad_message_removes_previous_assessment_and_valid_message_recovers(self):
        node, _, _, _ = self.construct()
        node.callback(Obj(data=json.dumps(snapshot())))
        self.assertEqual(node.status()["assessment_state"], "CANDIDATE_UNVERIFIED")
        node.callback(Obj(data="{"))
        result = node.status()
        self.assertEqual(result["assessment_state"], "INPUT_INVALID")
        self.assertEqual(result["observations"], {})
        node.callback(Obj(data=json.dumps(snapshot())))
        self.assertEqual(node.status()["assessment_state"], "CANDIDATE_UNVERIFIED")

    def test_invalid_settings_are_rejected(self):
        for params in [{"~publish_rate_hz": 0}, {"~input_timeout_s": -1}, {"~stopped_speed_mps": float("nan")}]:
            with self.assertRaises(ValueError):
                self.construct(params)

    def test_worker_publishes_assessment_only(self):
        node, _, _, _ = self.construct()
        messages = []
        node.pub = Obj(publish=lambda message: messages.append(json.loads(message.data)))
        node.rospy.is_shutdown = lambda: False
        node.shutdown = Obj(is_set=lambda: bool(messages), wait=lambda duration: None)
        node.run()
        self.assertEqual(len(messages), 1)
        self.assertTrue(messages[0]["assessment_only"])


class RecoveryWiringTests(unittest.TestCase):
    def read(self, relative):
        workspace = os.path.abspath(os.path.join(os.path.dirname(__file__), *([".."] * 6)))
        with open(os.path.join(workspace, relative)) as source:
            return source.read()

    def block(self, text, marker):
        start = text.index("{", text.index(marker))
        end, depth = start + 1, 1
        while depth:
            depth += (text[end] == "{") - (text[end] == "}")
            end += 1
        return text[start:end]

    def test_can_assessment_publisher_has_no_vehicle_actions(self):
        text = re.sub(r"//[^\n]*", "", self.read("ros2can/ros_canlib/src/vehicle/src/VehicleController.cpp"))
        body = self.block(text, "void VehicleController::publishVehicleRecoveryInputs()")
        for forbidden in ["state_ =", "switchState(", "emergency_stop(", "stopCAN(",
                          "startVIM(", "exitAD(", ".send", ".stop", ".monitor"]:
            self.assertNotIn(forbidden, body)
        self.assertIn(".getSignal(", body)
        self.assertIn("vehicle_recovery_inputs_pub_.publish", body)
        self.assertIn("steering_interrupt", body)
        self.assertIn("can_sample_freshness_verified", body)
        main = self.read("ros2can/ros_canlib/src/vehicle/src/main.cpp")
        self.assertIn('"/adeye/vehicle_recovery_inputs"', main)
        self.assertIn("vehicle_controller.publishVehicleRecoveryInputs();", main)

    def test_original_terminal_fault_and_intervention_behaviour_remain(self):
        text = self.read("ros2can/ros_canlib/src/vehicle/src/VehicleController.cpp")
        terminal = self.block(text, "void VehicleController::switchState(")
        self.assertIn("if(state_ != VCState::FAULT)", terminal)
        emergency = self.block(text, "void VehicleController::emergency_stop(")
        for expected in ["state_ = VCState::FAULT", "exitAD();", "stopCAN();"]:
            self.assertIn(expected, emergency)
        interrupt = self.block(text, "void VehicleController::checkDriverInterrupt()")
        self.assertIn('"DRIVER_BRAKE_INTERVENTION"', interrupt)
        self.assertIn('"DRIVER_ACCELERATOR_INTERVENTION"', interrupt)
        self.assertNotIn('"DRIVER_STEERING_INTERVENTION"', interrupt)
        entry = self.block(text, "void VehicleController::publishVehicleFaultStatus(")
        self.assertIn("state_ == VCState::FAULT && fault_entry_reason_.empty()", entry)

    def test_monitor_is_installed_and_default_launch_is_information_only(self):
        package = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        root = ET.parse(os.path.join(package, "launch", "manager_real_world.launch")).getroot()
        self.assertTrue(any(a.get("name") == "enable_vehicle_recovery_monitor" and a.get("default") == "true"
                            for a in root.findall("arg")))
        monitor = ET.parse(os.path.join(package, "launch", "vehicle_recovery_monitor.launch")).getroot()
        self.assertEqual(len(monitor.findall("node")), 1)
        self.assertEqual(monitor.find("node").get("type"), "vehicle_recovery_monitor.py")
        source = self.read("adeye-core/AD-EYE/ROS_Packages/src/AD-EYE/src/vehicle_recovery_monitor.py")
        tree = ast.parse(source)
        calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)]
        publisher_topics = [ast.literal_eval(n.args[0]) for n in calls if n.func.attr == "Publisher"]
        self.assertEqual(publisher_topics, ["/adeye/vehicle_recovery_status"])
        self.assertFalse(any(n.func.attr in ("Service", "ServiceProxy") for n in calls))
        self.assertIn("src/vehicle_recovery_monitor.py", self.read("adeye-core/AD-EYE/ROS_Packages/src/AD-EYE/CMakeLists.txt"))


if __name__ == "__main__":
    unittest.main()
