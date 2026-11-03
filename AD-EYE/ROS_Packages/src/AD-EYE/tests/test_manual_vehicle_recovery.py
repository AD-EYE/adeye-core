"""Check manual-reset steps and inspect the C++ code without building ROS locally."""
import json
import os
import re
import sys
import threading
import unittest
import xml.etree.ElementTree as ET
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))
import manual_vehicle_recovery as manual


class Obj(object):
    def __init__(self, **kwargs): self.__dict__.update(kwargs)


def sample(**changes):
    data = {"state": "FAULT", "reason": "DRIVER_BRAKE_INTERVENTION", "fault_entry_reason": "DRIVER_BRAKE_INTERVENTION",
            "manual_recovery_enabled": True, "boot_id": "123-456", "speed_mps": 0,
            "ad_mode_state": 0, "brake_interrupt": 0, "accelerator_interrupt": 0, "steering_interrupt": 0}
    data.update(changes)
    return data


class ResetPolicyTests(unittest.TestCase):
    def test_explicit_confirmation_and_supported_software_fault_required(self):
        manual.validate_reset(sample(), True)
        for confirmation in [False, None, 1, "true"]:
            with self.assertRaises(ValueError): manual.validate_reset(sample(), confirmation)
        for changes in [{"state": "ENGAGED"}, {"state": "INITIALIZING"}, {"reason": "CAN_SIGNAL_INVALID"},
                        {"fault_entry_reason": "WHEEL_LOCK_FAILED"}, {"manual_recovery_enabled": False}, {"boot_id": ""}]:
            with self.assertRaises(ValueError): manual.validate_reset(sample(**changes), True)

    def test_stop_and_accelerator_faults_are_identified_software_reset_candidates(self):
        for reason in ["STOP_COMMAND", "DRIVER_ACCELERATOR_INTERVENTION"]:
            manual.validate_reset(sample(reason=reason, fault_entry_reason=reason), True)

    def test_missing_motion_or_intervention_evidence_refuses_reset(self):
        for changes in [{"speed_mps": None}, {"speed_mps": float("nan")}, {"speed_mps": 1}, {"speed_mps": -1},
                        {"ad_mode_state": 1}, {"ad_mode_state": 3}, {"brake_interrupt": 1},
                        {"accelerator_interrupt": None}, {"steering_interrupt": 1}]:
            with self.assertRaises(ValueError): manual.validate_reset(sample(**changes), True)

    def test_new_command_age_watchdogs_are_not_used(self):
        manual.validate_reset(sample(steering_command_age_s=10000, acceleration_command_age_s=10000,
                                     wall_time_s=1, can_sample_freshness_verified=False), True)

    def test_vehicle_not_ready_does_not_prevent_software_startup_attempt(self):
        # Readiness is not used to claim that an OEM fault was cleared.
        manual.validate_reset(sample(ad_activation_ok=0), True)


class ResetOrderingTests(unittest.TestCase):
    def backend(self, fail=None):
        trace = []
        def step(name):
            trace.append(name)
            if fail == name: raise RuntimeError("step failed: " + name)
        return Obj(prepare=lambda: step("prepare"), restart=lambda: step("restart"),
                   initialize=lambda: step("initialize"), release=lambda: step("release"),
                   abort=lambda detail: trace.append("abort")), trace

    def test_cleanup_must_precede_restart_and_initialization(self):
        backend, trace = self.backend()
        success, detail = manual.run_reset(backend)
        self.assertTrue(success)
        self.assertEqual(trace, ["prepare", "restart", "initialize", "release"])
        self.assertIn("INITIALIZING", detail)

    def test_every_failed_step_aborts_without_running_later_steps(self):
        stages = ["prepare", "restart", "initialize", "release"]
        for stage in stages:
            backend, trace = self.backend(stage)
            success, detail = manual.run_reset(backend)
            self.assertFalse(success)
            self.assertEqual(trace, stages[:stages.index(stage) + 1] + ["abort"])

    def test_stale_acknowledgements_are_ignored(self):
        node = manual.ManualVehicleRecovery.__new__(manual.ManualVehicleRecovery)
        node.lock = threading.RLock(); node.token = "aabb"; node.acks = {}; node.failed_ack = False
        node.ack(Obj(data=json.dumps({"component": "/vi_controller", "token": "old", "phase": "prepared"})))
        self.assertEqual(node.acks, {})
        node.ack(Obj(data=json.dumps({"component": "/vi_controller", "token": "aabb", "phase": "prepared"})))
        self.assertIn(("/vi_controller", "prepared"), node.acks)
        node.ack(Obj(data=json.dumps({"component": "/vi_controller", "token": "aabb", "phase": "failed"})))
        self.assertTrue(node.failed_ack)

    def test_pending_gui_panels_are_tracked_without_age_based_auto_actions(self):
        node = manual.ManualVehicleRecovery.__new__(manual.ManualVehicleRecovery)
        node.lock = threading.RLock(); node.pending_gui = set()
        node.gui_state(Obj(data=json.dumps({"component": "gui-123", "pending": True})))
        self.assertEqual(node.pending_gui, {"gui-123"})
        node.gui_state(Obj(data=json.dumps({"component": "gui-123", "pending": False})))
        self.assertEqual(node.pending_gui, set())

    def test_prepare_requires_every_actor_and_registered_pending_gui_ack(self):
        node = manual.ManualVehicleRecovery.__new__(manual.ManualVehicleRecovery)
        node.required = {"/vi_controller", "/steering_fault_proxy_live", "gui-123"}
        node.acks = {}; node.snapshot = sample()
        node.stage = lambda *args: None
        def wait(predicate, action=None):
            self.assertEqual(action, "PREPARE")
            self.assertFalse(predicate())
            for component in node.required:
                node.acks[(component, "prepared")] = {}
            self.assertTrue(predicate())
        node.wait = wait
        node.prepare()

    def test_release_does_not_wait_for_a_gui_to_rearm_or_send_commands(self):
        node = manual.ManualVehicleRecovery.__new__(manual.ManualVehicleRecovery)
        node.required = {"/vi_controller", "/vehicle_controller", "gui-123"}
        node.acks = {("/vi_controller", "released"): {}, ("/vehicle_controller", "released"): {}}
        node.stage = lambda *args: None
        node.wait = lambda predicate, action: self.assertTrue(predicate())
        params = []
        node.rospy = Obj(set_param=lambda key, value: params.append((key, value)))
        node.release()
        self.assertEqual(params, [(manual.TOKEN_PARAM, "")])


class ResetSourceGuards(unittest.TestCase):
    def read(self, name):
        root = os.path.abspath(os.path.join(os.path.dirname(__file__), *([".."] * 6)))
        with open(os.path.join(root, name)) as source: return source.read()

    def block(self, text, marker):
        start = text.index("{", text.index(marker)); depth = 1; end = start + 1
        while depth:
            depth += (text[end] == "{") - (text[end] == "}"); end += 1
        return text[start:end]

    def test_replacement_starts_held_and_only_initializes_on_explicit_stage(self):
        main = self.read("ros2can/ros_canlib/src/vehicle/src/main.cpp")
        self.assertIn("if (!vehicle_controller.recoveryHeld()) vehicle_controller.initalizeCAN()", main)
        text = self.read("ros2can/ros_canlib/src/vehicle/src/VehicleController.cpp")
        callback = self.block(text, "void VehicleController::recoveryControlCallback(")
        self.assertIn("recovery_held_ = true", callback)
        self.assertIn("initalizeCAN()", callback)
        self.assertNotIn("startVIM(", callback)
        self.assertNotIn("enterAD(", callback)
        self.assertNotIn("state_ = VCState::ENABLED", callback)
        self.assertIn("state_ == VCState::INITIALIZING", callback)
        self.assertIn("recovery_needs_ad_request_ = true", callback)
        self.assertIn("recovery_boot_ = false", callback)
        state = self.block(text, "void VehicleController::handleStateEvents()")
        self.assertIn("if (recovery_held_) return", state)
        self.assertIn("!recovery_needs_ad_request_", state)

    def test_both_driving_commands_and_all_gui_commands_are_held(self):
        text = self.read("ros2can/ros_canlib/src/vehicle/src/VehicleController.cpp")
        for method in ["scSteer(double", "scAccelerate(double", "scEnterAD()"]:
            self.assertIn("if (recovery_held_) return", self.block(text, "void VehicleController::" + method))
        hvi = self.read("ros2can/ros_canlib/src/vehicle/src/HVIController.cpp")
        self.assertIn("vh_.recoveryHeld() && command != HVICommand::STOP_command", hvi)

    def test_fault_cleanup_does_not_run_recovery_to_old_commands(self):
        base = "adeye-core/AD-EYE/ROS_Packages/src/AD-EYE/src/"
        for name in ["steering_fault_proxy.cpp", "recorded_acceleration_fault_proxy.cpp"]:
            text = self.read(base + name)
            cleanup = self.block(text, "new RecoveryGate(nh_")
            for field in ["active_fault_ = Fault::NONE", "has_latest_input_ = has_last_published_ = false", "constant_duration_s_ = 0.0"]:
                self.assertIn(field, cleanup)
            for forbidden in ["publishOutput(", "publishRecordedOutput(", "beginRecovery("]:
                self.assertNotIn(forbidden, cleanup)
            for method in ["inputCallback(", "commandCallback(", "timerCallback("]:
                self.assertIn("recovery_gate_->blocked()", self.block(text, "void " + method))

    def test_live_cached_targets_are_cleared_and_new_goal_is_required(self):
        base = "adeye-core/AD-EYE/ROS_Packages/src/AD-EYE/"
        for name in ["vehicle_controller.cpp", "ctrl_cmd_republisher.cpp", "control_switch.cpp"]:
            text = self.read(base + "src/" + name)
            self.assertIn("RecoveryGate", text)
            self.assertIn("}, true)", text)
            self.assertIn("blocked()", text)
        text = self.read(base + "src/vehicle_controller.cpp")
        self.assertIn("accelPIDController.resetForRecovery()", text)
        header = self.read(base + "include/recovery_gate.h")
        self.assertIn("waiting_goal_ = new_goal_", header)
        self.assertIn("if (!held_ && waiting_goal_)", header)
        self.assertIn("if (!enabled_) return", header)

    def test_opt_in_flag_is_forwarded_by_all_module_launches(self):
        package = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        import glob
        for filename in glob.glob(os.path.join(package, "launch", "manager_real_world*.launch")):
            root = ET.parse(filename).getroot()
            args = {a.get("name"): a for a in root.findall("arg")}
            self.assertEqual(args["enable_manual_recovery"].get("default"), "false", filename)
            if os.path.basename(filename) != "manager_real_world.launch":
                self.assertTrue(any(a.get("name") == "enable_manual_recovery" for a in root.find("include").findall("arg")), filename)
        source = self.read("adeye-core/AD-EYE/ROS_Packages/src/AD-EYE/src/manual_vehicle_recovery.py")
        self.assertIn("rosnode.kill_nodes", source)
        self.assertIn("os.kill(self.old_pid, 0)", source)
        self.assertNotIn("SIGKILL", source)
        self.assertIn("self.old_boot.startswith(str(pid)", source)


if __name__ == "__main__": unittest.main()
