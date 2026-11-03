"""Logic and input tests; no ROS master, CAN connection or car required."""
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
import physical_goal_stop_monitor as monitor


class Obj(object):
    def __init__(self, **kwargs):
        self.__dict__.update(kwargs)


class Stamp(object):
    def __init__(self, value=0):
        self.value = value

    def to_sec(self):
        return self.value

    @staticmethod
    def now():
        return Stamp(100)


def pose(x=0, frame="map", stamp=100):
    return Obj(header=Obj(frame_id=frame, stamp=Stamp(stamp)),
               pose=Obj(position=Obj(x=x, y=0, z=0),
                        orientation=Obj(x=0, y=0, z=0, w=1)))


class ProgressTests(unittest.TestCase):
    def test_approach_and_speed_dependent_distance(self):
        p = monitor.GoalProgress()
        self.assertFalse(p.update(100, 5, 0)["would_request_stop"])
        self.assertGreater(p.update(20, 5, 1)["approach_distance_m"], 20)
        self.assertEqual(p.state, "APPROACHING")

    def test_zero_speed_alone_is_not_arrival(self):
        p = monitor.GoalProgress()
        self.assertEqual(p.update(10, 0, 0)["progress_state"], "DRIVING")
        self.assertEqual(p.update(10, 0, 10)["progress_state"], "DRIVING")

    def test_arrival_requires_speed_and_time(self):
        p = monitor.GoalProgress()
        self.assertEqual(p.update(0.5, 2, 0)["progress_state"], "STOPPING")
        self.assertEqual(p.update(0.5, 0, 1)["progress_state"], "STOPPING")
        self.assertEqual(p.update(0.5, 0, 2)["progress_state"], "STOPPED")
        self.assertEqual(p.update(0.5, 1, 3)["progress_state"], "STOPPING")

    def test_position_noise_does_not_resume_driving(self):
        p = monitor.GoalProgress()
        p.update(0.5, 1, 0)
        self.assertEqual(p.update(10, 1, 1)["suggested_speed_limit_mps"], 0)

    def test_input_gap_resets_standstill_confirmation(self):
        p = monitor.GoalProgress()
        p.update(0.5, 0, 0)
        p.unavailable()
        self.assertEqual(p.update(0.5, 0, 10)["progress_state"], "STOPPING")
        self.assertEqual(p.update(0.5, 0, 11)["progress_state"], "STOPPED")

    def test_new_goal_resets_progress(self):
        p = monitor.GoalProgress()
        p.update(0, 0, 0)
        p.update(0, 0, 1)
        p.reset()
        self.assertFalse(p.update(20, 0, 2)["would_request_stop"])

    def test_reverse_speed_is_not_standstill(self):
        p = monitor.GoalProgress()
        p.update(0, -1, 0)
        self.assertEqual(p.update(0, -1, 10)["progress_state"], "STOPPING")

    def test_invalid_inputs(self):
        with self.assertRaises(ValueError):
            monitor.GoalProgress(deceleration=0)
        with self.assertRaises(ValueError):
            monitor.GoalProgress().update(float("nan"), 0, 0)


class InputTests(unittest.TestCase):
    def setUp(self):
        self.saved_time = monitor.time.time
        monitor.time.time = lambda: 100
        self.node = monitor.PhysicalGoalStopMonitor.__new__(monitor.PhysicalGoalStopMonitor)
        n = self.node
        n.lock = threading.RLock()
        n.rospy = Obj(Time=Stamp)
        n.tf = Obj(LookupException=LookupError, ConnectivityException=ConnectionError,
                   ExtrapolationException=RuntimeError)
        n.frame = "map"
        n.listener = Obj(transformPose=lambda frame, message: pose(message.pose.position.x + 10, frame))
        n.input_timeout, n.ad_timeout, n.replay = 2, 5, False
        n.control_enabled = False
        n.progress = monitor.GoalProgress()
        n.speed_profile = monitor.GoalSpeedProfile(n.progress.deceleration)
        n.goal, n.pose = pose(20), pose(0)
        n.speed = Obj(header=Obj(stamp=Stamp(100)), twist=Obj(linear=Obj(x=1)))
        n.pose_received = n.speed_received = n.ad_received = 100
        n.ad = {"state": "ENGAGED", "wall_time_s": 100}
        n.goal_id = 1
        n.new_goal_pending = False
        n.goal_error = n.pose_error = n.speed_error = n.ad_error = None

    def tearDown(self):
        monitor.time.time = self.saved_time

    def test_valid_observation_and_only_status_publisher(self):
        result = self.node.status()
        self.assertEqual(result["state"], "DRIVING")
        self.assertTrue(result["monitor_only"])
        self.assertFalse(result["actuator_commands_enabled"])
        with open(monitor.__file__.replace(".pyc", ".py")) as source:
            tree = ast.parse(source.read())
        publishers = [node for node in ast.walk(tree) if isinstance(node, ast.Call)
                      and isinstance(node.func, ast.Attribute) and node.func.attr == "Publisher"]
        self.assertEqual(set(ast.literal_eval(call.args[0]) for call in publishers),
                         set(["/adeye/goal_stop_status", "/adeye/goal_stop_speed_limit", "/adeye/vehicle_recovery_ack"]))

    def test_waiting_for_goal_does_not_stop(self):
        self.node.goal = None
        self.assertEqual(self.node.status()["state"], "WAITING_FOR_GOAL")
        self.assertFalse(self.node.status()["would_request_stop"])

    def test_tf_changes_distance_and_missing_tf_blocks_arrival(self):
        self.node.goal = pose(20, "world")
        self.assertEqual(self.node.status()["distance_to_goal_m"], 30)
        def missing(frame, message):
            raise LookupError("missing transform")
        self.node.listener.transformPose = missing
        self.assertIn("TF_UNAVAILABLE", self.node.status()["input_issues"])
        self.assertFalse(self.node.status()["would_request_stop"])

    def test_old_pose_and_old_message_stamp(self):
        self.node.pose_received = 90
        result = self.node.status()
        self.assertIn("POSE_OUTDATED", result["freshness_warnings"])
        self.assertEqual(result["state"], "DRIVING")
        self.assertTrue(result["uses_last_known_inputs"])
        self.node.pose_received = 100
        self.node.pose.header.stamp = Stamp(90)
        result = self.node.status()
        self.assertIn("POSE_TIMESTAMP_INVALID_OR_OUTDATED", result["freshness_warnings"])
        self.assertEqual(result["state"], "DRIVING")

    def test_latched_old_ad_status_and_manual_mode(self):
        self.node.ad["wall_time_s"] = 90
        result = self.node.status()
        self.assertIn("AD_STATUS_OUTDATED", result["freshness_warnings"])
        self.assertEqual(result["state"], "DRIVING")
        self.node.ad["wall_time_s"] = 100
        self.node.ad["state"] = "ENABLED"
        self.assertIn("AD_NOT_ENGAGED", self.node.status()["input_issues"])

    def test_freshness_does_not_suppress_monitor_calculation(self):
        self.node.goal = pose(0)
        self.node.pose_received = self.node.speed_received = 90
        self.node.ad["wall_time_s"] = 90
        result = self.node.status()
        self.assertEqual(result["state"], "STOPPING")
        self.assertTrue(result["would_request_stop"])
        self.assertFalse(result["actuator_commands_enabled"])
        self.assertTrue(result["freshness_checks_informational_only"])
        self.assertEqual(len(result["freshness_warnings"]), 3)

    def test_replay_has_no_stop_recommendation(self):
        self.node.replay = True
        self.node.goal = pose(0)
        self.assertIn("REPLAY_MODE_DISABLED", self.node.status()["input_issues"])
        self.assertFalse(self.node.status()["would_request_stop"])

    def test_bad_status_and_new_goal(self):
        self.node.ad_callback(Obj(data="[]"))
        self.assertEqual(self.node.ad_error, "INVALID_AD_STATUS")
        self.node.ad_callback(Obj(data=json.dumps({"state": "ENGAGED", "wall_time_s": "100"})))
        self.assertIsNone(self.node.ad_error)
        self.assertEqual(self.node.status()["ad_status_age_s"], 0)
        self.node.goal_callback(pose(40))
        self.assertEqual(self.node.goal_id, 2)
        self.assertEqual(self.node.progress.state, "DRIVING")

    def test_monitor_only_never_creates_speed_cap(self):
        self.node.goal = pose(0)
        result = self.node.status()
        self.assertTrue(result["would_request_stop"])
        self.assertIsNone(self.node.control_limit(result))
        self.assertFalse(result["goal_stop_latched"])

    def test_enabled_goal_stop_ramps_and_releases_for_new_goal(self):
        self.node.control_enabled = True
        self.node.goal = pose(0)
        self.node.speed.twist.linear.x = 2
        result = self.node.status()
        self.assertEqual(self.node.control_limit(result), 2)
        monitor.time.time = lambda: 100.1
        result = self.node.status()
        self.assertAlmostEqual(self.node.control_limit(result), 1.95)
        self.assertFalse(result["brake_hold_requested"])
        self.node.goal_callback(pose(100))
        monitor.time.time = lambda: 100.2
        result = self.node.status()
        self.assertEqual(self.node.control_limit(result), -1.0)
        self.assertFalse(result["goal_stop_latched"])
        self.assertTrue(result["released_for_new_goal"])
        self.assertIsNone(self.node.control_limit(self.node.status()))

    def start_stop(self):
        self.node.control_enabled = True
        self.node.goal = pose(0)
        self.node.control_limit(self.node.status())

    def test_invalid_goal_does_not_release(self):
        self.start_stop()
        self.node.goal_callback(pose(float("nan")))
        result = self.node.status()
        self.assertGreaterEqual(self.node.control_limit(result), 0)
        self.assertTrue(result["goal_stop_latched"])
        self.assertFalse(result["released_for_new_goal"])

    def test_new_goal_waits_for_transform_before_release(self):
        self.start_stop()
        self.node.goal_callback(pose(100, "world"))
        transform = self.node.listener.transformPose
        def missing(frame, message):
            raise LookupError("no transform")
        self.node.listener.transformPose = missing
        result = self.node.status()
        self.assertGreaterEqual(self.node.control_limit(result), 0)
        self.assertTrue(result["new_goal_pending"])
        self.node.listener.transformPose = transform
        self.assertEqual(self.node.control_limit(self.node.status()), -1)

    def test_mode_change_does_not_release_old_stop(self):
        self.start_stop()
        self.node.ad["state"] = "ENABLED"
        result = self.node.status()
        self.assertGreaterEqual(self.node.control_limit(result), 0)
        self.assertFalse(result["released_for_new_goal"])
        self.node.goal_callback(pose(100))
        result = self.node.status()
        self.assertGreaterEqual(self.node.control_limit(result), 0)
        self.assertTrue(result["new_goal_pending"])
        self.node.ad["state"] = "ENGAGED"
        self.assertEqual(self.node.control_limit(self.node.status()), -1)

    def test_old_inputs_are_only_warnings_and_do_not_block_new_goal(self):
        self.start_stop()
        self.node.goal_callback(pose(100))
        self.node.pose_received = self.node.speed_received = 0
        self.node.ad["wall_time_s"] = 0
        result = self.node.status()
        self.assertTrue(result["freshness_warnings"])
        self.assertEqual(self.node.control_limit(result), -1)

    def test_nearby_new_goal_releases_then_can_request_a_new_stop(self):
        self.start_stop()
        self.node.goal_callback(pose(0))
        self.assertEqual(self.node.control_limit(self.node.status()), -1)
        self.assertGreaterEqual(self.node.control_limit(self.node.status()), 0)

    def test_monitor_and_replay_never_send_release(self):
        for replay in [False, True]:
            self.node.control_enabled = False
            self.node.replay = replay
            self.node.goal_callback(pose(100))
            self.assertIsNone(self.node.control_limit(self.node.status()))

    def test_worker_publishes_release_on_the_latched_limit_topic(self):
        self.start_stop()
        self.node.goal_callback(pose(100))
        limits, statuses = [], []
        self.node.shutdown = Obj(is_set=lambda: bool(statuses), wait=lambda duration: None)
        self.node.rospy.is_shutdown = lambda: False
        self.node.Float64 = self.node.String = Obj
        self.node.limit_pub = Obj(publish=lambda message: limits.append(message.data))
        self.node.pub = Obj(publish=lambda message: statuses.append(json.loads(message.data)))
        self.node.rate = 10
        self.node.run()
        self.assertEqual(limits, [-1.0])
        self.assertFalse(statuses[0]["goal_stop_latched"])
        self.assertEqual(statuses[0]["reason"], "NEW_GOAL_RELEASED")


class SpeedProfileTests(unittest.TestCase):
    def test_explicit_reset_allows_next_goal_to_have_its_own_cap(self):
        p = monitor.GoalSpeedProfile(0.5)
        p.update(0, 0, 0)
        p.reset()
        self.assertIsNone(p.update(None, None, 1))
        self.assertEqual(p.update(1, 2, 2), 2)

    def test_no_request_never_starts_stopping(self):
        p = monitor.GoalSpeedProfile(0.5)
        self.assertIsNone(p.update(None, None, 0))
        self.assertIsNone(p.update(None, 2, 100))

    def test_target_reaches_zero_gradually_and_stays_there(self):
        p = monitor.GoalSpeedProfile(0.5)
        self.assertEqual(p.update(0, 2, 0), 2)
        previous = 2
        for step in range(1, 51):
            value = p.update(0, 2, step * 0.1)
            self.assertLessEqual(value, previous)
            self.assertLessEqual(previous - value, 0.05000001)
            previous = value
        self.assertEqual(previous, 0)
        self.assertEqual(p.update(10, 2, 6), 0)
        self.assertEqual(p.update(None, None, 7), 0)

    def test_cap_is_not_raised_when_distance_or_speed_increases(self):
        p = monitor.GoalSpeedProfile(0.5)
        p.update(1, 2, 0)
        self.assertAlmostEqual(p.update(1, 2, 0.2), 1.9)
        self.assertAlmostEqual(p.update(4, 4, 0.4), 1.8)

    def test_pause_does_not_create_a_large_target_step(self):
        p = monitor.GoalSpeedProfile(0.5)
        p.update(0, 2, 0)
        self.assertEqual(p.update(None, None, 100), 1.75)

    def test_invalid_limits_are_rejected(self):
        p = monitor.GoalSpeedProfile(0.5)
        for bad in [-1, float("nan"), float("inf")]:
            with self.assertRaises(ValueError):
                p.update(bad, 1, 0)

    def test_reverse_speed_uses_magnitude(self):
        p = monitor.GoalSpeedProfile(0.5)
        self.assertEqual(p.update(0, -2, 0), 2)

    def test_standstill_does_not_raise_target(self):
        p = monitor.GoalSpeedProfile(0.5)
        self.assertEqual(p.update(0, 0, 0), 0)


class LaunchModeTests(unittest.TestCase):
    def construct(self, enabled, replay):
        publishers = []
        rospy = types.ModuleType("rospy")
        params = {"~enable_speed_control": enabled, "~replay_mode": replay}
        rospy.get_param = lambda name, default=None: params.get(name, default)
        rospy.Publisher = lambda topic, *args, **kwargs: publishers.append(topic) or Obj()
        rospy.Subscriber = lambda *args, **kwargs: Obj()
        rospy.on_shutdown = lambda callback: None
        rospy.loginfo = lambda *args: None
        tf = types.ModuleType("tf")
        tf.TransformListener = lambda: Obj()
        geometry = types.ModuleType("geometry_msgs")
        geometry.msg = types.ModuleType("geometry_msgs.msg")
        geometry.msg.PoseStamped = geometry.msg.TwistStamped = Obj
        std = types.ModuleType("std_msgs")
        std.msg = types.ModuleType("std_msgs.msg")
        std.msg.String = std.msg.Float64 = Obj
        replacements = {"rospy": rospy, "tf": tf, "geometry_msgs": geometry,
                        "geometry_msgs.msg": geometry.msg, "std_msgs": std,
                        "std_msgs.msg": std.msg}
        saved = {key: sys.modules.get(key) for key in replacements}
        thread = monitor.threading.Thread
        try:
            sys.modules.update(replacements)
            monitor.threading.Thread = lambda **kwargs: Obj(start=lambda: None)
            node = monitor.PhysicalGoalStopMonitor()
            return node, publishers
        finally:
            monitor.threading.Thread = thread
            for key, value in saved.items():
                if value is None:
                    sys.modules.pop(key, None)
                else:
                    sys.modules[key] = value

    def test_monitor_only_has_no_control_publisher(self):
        node, publishers = self.construct(False, False)
        self.assertFalse(node.control_enabled)
        self.assertEqual(publishers, ["/adeye/goal_stop_status"])

    def test_live_opt_in_adds_only_speed_limit_publisher(self):
        node, publishers = self.construct(True, False)
        self.assertTrue(node.control_enabled)
        self.assertEqual(publishers, ["/adeye/goal_stop_status", "/adeye/goal_stop_speed_limit"])

    def test_replay_disables_control_even_when_enabled(self):
        node, publishers = self.construct(True, True)
        self.assertFalse(node.control_enabled)
        self.assertEqual(publishers, ["/adeye/goal_stop_status"])


class FreshnessSourceTests(unittest.TestCase):
    """Check that C++ warning code stays information-only without running ROS or CAN."""

    def read(self, filename):
        workspace = os.path.abspath(os.path.join(os.path.dirname(__file__), *([".."] * 6)))
        with open(os.path.join(workspace, filename)) as source:
            return re.sub(r"//[^\n]*", "", source.read())

    def block(self, text, marker):
        start = text.index("{", text.index(marker))
        depth = 1
        end = start + 1
        while depth:
            if text[end] == "{":
                depth += 1
            elif text[end] == "}":
                depth -= 1
            end += 1
        return text[start:end]

    def test_can_watchdogs_only_warn(self):
        text = self.read("ros2can/ros_canlib/src/vehicle/src/VehicleController.cpp")
        for name in ["checkSteeringCommandWatchdog", "checkAccelerationCommandWatchdog"]:
            body = self.block(text, "void VehicleController::" + name)
            self.assertIn("ROS_WARN_THROTTLE", body)
            for action in ["inhibitSteeringOutput(", "inhibitAccelerationOutput(",
                           "stopSignalGroup(", "emergency_stop(", "publishVehicleFaultStatus("]:
                self.assertNotIn(action, body)

    def test_proxy_age_warning_does_not_change_output(self):
        base = "adeye-core/AD-EYE/ROS_Packages/src/AD-EYE/src/"
        for filename in ["steering_fault_proxy.cpp", "recorded_acceleration_fault_proxy.cpp"]:
            text = self.read(base + filename)
            timer = self.block(text, "void timerCallback(")
            self.assertNotIn("zero_on_source_loss_", timer)
            age_branch = self.block(timer, "else if (!inputFresh())")
            self.assertIn("ROS_WARN_THROTTLE", age_branch)
            for action in ["publishOutput(", "enterInhibited(", "state_ =", "active_fault_ =", "return;"]:
                self.assertNotIn(action, age_branch)

    def test_legacy_recovery_does_not_inhibit_for_age(self):
        text = self.read("adeye-core/AD-EYE/ROS_Packages/src/AD-EYE/src/ctrl_cmd_republisher.cpp")
        callback = self.block(text, "void steeringFaultWatchdogCallback(")
        self.assertNotIn("steering_output_inhibit_pub.publish", callback)
        self.assertIn("if (has_last_unfaulted_steering)", callback)

    def test_speed_cap_is_applied_each_pid_tick_and_never_released_by_age(self):
        text = self.read("adeye-core/AD-EYE/ROS_Packages/src/AD-EYE/src/vehicle_controller.cpp")
        body = self.block(text, "void updatePID()")
        self.assertIn("std::min(std::abs(target), goal_speed_limit_)", body)
        self.assertLess(body.index("setTarget(target)"), body.index("accelPIDController.tick()"))
        callback = self.block(text, "void goalSpeedLimitCallback(")
        self.assertIn("std::min(goal_speed_limit_, msg->data)", callback)
        self.assertNotIn("WallTime", callback)
        release = self.block(callback, "if (msg->data == -1.0)")
        self.assertIn("has_goal_speed_limit_ = false", release)
        self.assertIn("linear_speed_target_ = 0.0", release)
        self.assertIn("return;", release)

    def test_goal_acceleration_is_published_before_legacy_fault_processing(self):
        text = self.read("adeye-core/AD-EYE/ROS_Packages/src/AD-EYE/src/vehicle_controller.cpp")
        body = self.block(text, "void sendAcceleration(double req)")
        goal = self.block(body, "if (goal_stop_enabled_ && has_goal_speed_limit_)")
        self.assertIn("goal_command.data = req", goal)
        self.assertIn("pub_goal_stop_acceleration.publish(goal_command)", goal)
        self.assertIn("return;", goal)
        self.assertLess(body.index("pub_goal_stop_acceleration.publish"), body.index("clearExpiredAccelerationFaults()"))

    def test_proxy_priority_precedes_all_fault_timers_and_recovery(self):
        base = "adeye-core/AD-EYE/ROS_Packages/src/AD-EYE/src/"
        for filename in ["steering_fault_proxy.cpp", "recorded_acceleration_fault_proxy.cpp"]:
            text = self.read(base + filename)
            timer = self.block(text, "void timerCallback(")
            self.assertLess(timer.index("if (goal_stop_active_)"), timer.index("State::FAULT_ACTIVE"))
            branch = self.block(timer, "if (goal_stop_active_)")
            self.assertIn("return;", branch)
            for action in ["faultedOutput(", "beginRecovery(", "stopConstant("]:
                self.assertNotIn(action, branch)
            command = self.block(text, "void commandCallback(")
            self.assertLess(command.index("if (goal_stop_active_"), command.index("parseConstantCommand("))
            rejection = self.block(command, "if (goal_stop_active_")
            self.assertIn('"rejected"', rejection)
            self.assertIn("return;", rejection)

    def test_acceleration_goal_priority_skips_stored_nominal_and_fault_bounds(self):
        text = self.read("adeye-core/AD-EYE/ROS_Packages/src/AD-EYE/src/recorded_acceleration_fault_proxy.cpp")
        latch = self.block(text, "void latchGoalStop()")
        self.assertIn("active_fault_ = Fault::NONE", latch)
        self.assertIn("constant_duration_s_ = 0.0", latch)
        self.assertNotIn("beginRecovery(", latch)
        goal = self.block(text, "void goalAccelerationCallback(")
        self.assertNotIn("latchGoalStop()", goal)
        self.assertIn("return;", self.block(goal, "if (!goal_stop_active_)"))
        self.assertIn("publishRecordedOutput(goal_acceleration_)", goal)
        self.assertNotIn("publishFaultedOutput(", goal)
        nominal = self.block(text, "void inputCallback(")
        guard = self.block(nominal, "if (goal_stop_active_)")
        self.assertIn("return;", guard)
        self.assertNotIn("publish", guard)

    def test_steering_returns_gradually_instead_of_zeroing(self):
        text = self.read("adeye-core/AD-EYE/ROS_Packages/src/AD-EYE/src/steering_fault_proxy.cpp")
        latch = self.block(text, "void goalLimitCallback(")
        self.assertIn("goal_steering_value_ = last_can_range_output_", latch)
        self.assertIn("active_fault_ = Fault::NONE", latch)
        timer = self.block(text, "void timerCallback(")
        transition = self.block(timer, "if (goal_stop_active_)")
        self.assertIn("max_recovery_rate_rad_s_ * std::min(elapsed_s, 0.1)", transition)
        self.assertIn("latest_input_ - goal_steering_value_", transition)
        self.assertNotIn("publishOutput(0.0)", transition)

    def test_can_accepts_only_goal_acceleration_during_stop(self):
        text = self.read("ros2can/ros_canlib/src/vehicle/src/VehicleController.cpp")
        body = self.block(text, "void VehicleController::scAccelerate(")
        guard = self.block(body, "if (goal_stop_active_ && !from_goal_stop)")
        self.assertIn("return;", guard)
        self.assertLess(body.index("if (goal_stop_active_"), body.index("accelerate(value)"))
        callback = self.block(text, "void VehicleController::goalStopAccelerationCallback(")
        self.assertNotIn("goal_stop_active_ = true", callback)
        self.assertIn("return;", self.block(callback, "if (!goal_stop_active_)"))
        self.assertIn("scAccelerate(msg->data, true)", callback)
        header = self.read("ros2can/ros_canlib/src/vehicle/src/include/VehicleController.h")
        for name in ["directSteeringCommandsAllowed", "directAccelerationCommandsAllowed"]:
            self.assertIn("!goal_stop_active_", self.block(header, name + "()"))

    def test_live_only_launch_priority_and_explicit_opt_in(self):
        package = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
        root = ET.parse(os.path.join(package, "launch", "manager_real_world.launch")).getroot()
        nodes = {node.get("name"): node for node in root.findall("node")}
        for name in ["steering_fault_proxy_live", "acceleration_fault_proxy_live"]:
            value = next(p.get("value") for p in nodes[name].findall("param")
                         if p.get("name") == "enable_goal_stop_priority")
            expr = value[len("$(eval "):-1]
            for enabled, steering, acceleration in [(False, False, False), (True, False, False),
                                                     (True, True, False), (True, False, True), (True, True, True)]:
                args = {"enable_goal_stop": enabled, "use_recorded_steering": steering,
                        "use_recorded_acceleration": acceleration}
                actual = eval(expr, {"__builtins__": {}}, {"arg": args.__getitem__})
                self.assertEqual(actual, enabled and not steering and not acceleration)
        for name in ["steering_fault_proxy_recorded", "recorded_acceleration_fault_proxy"]:
            self.assertFalse(any(p.get("name") == "enable_goal_stop_priority"
                                 for p in nodes[name].findall("param")))
        for filename in ["manager_real_world_steering_faults.launch", "manager_real_world_acceleration_faults.launch",
                         "manager_real_world_steering_acceleration_faults.launch"]:
            launch = ET.parse(os.path.join(package, "launch", filename)).getroot()
            self.assertTrue(any(a.get("name") == "enable_goal_stop" and a.get("default") == "false"
                                for a in launch.findall("arg")))
            self.assertTrue(any(a.get("name") == "enable_goal_stop" and a.get("value") == "$(arg enable_goal_stop)"
                                for a in launch.find("include").findall("arg")))

    def test_release_reaches_both_proxies_without_restoring_faults(self):
        base = "adeye-core/AD-EYE/ROS_Packages/src/AD-EYE/src/"
        for filename, cached in [("steering_fault_proxy.cpp", "has_goal_steering_"),
                                 ("recorded_acceleration_fault_proxy.cpp", "has_goal_acceleration_")]:
            callback = self.block(self.read(base + filename), "void goalLimitCallback(")
            release = self.block(callback, "message->data == -1.0")
            self.assertIn("goal_stop_active_ = false", release)
            self.assertIn(cached + " = false", release)
            for action in ["active_fault_ =", "publishOutput(", "beginRecovery("]:
                self.assertNotIn(action, release)

    def test_can_release_restores_policy_without_sending_a_car_command(self):
        text = self.read("ros2can/ros_canlib/src/vehicle/src/VehicleController.cpp")
        callback = self.block(text, "void VehicleController::goalStopLimitCallback(")
        release = self.block(callback, "msg->data == -1.0")
        self.assertIn("goal_stop_active_ = false", release)
        self.assertIn("publishVehicleFaultStatus(", release)
        for action in ["scAccelerate(", "accelerate(", "emergency_stop(", "steer("]:
            self.assertNotIn(action, release)


if __name__ == "__main__":
    unittest.main()
