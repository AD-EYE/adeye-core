#!/usr/bin/env python
"""Watch progress towards the RViz goal and optionally lower the requested speed.

/adeye/goal_stop_speed_limit carries a speed limit of zero or more, in m/s.
Use -1 to remove the previous goal's limit, not to request reverse movement.
"""

import copy
import json
import math
import threading
import time


def finite(value):
    return not math.isnan(value) and not math.isinf(value)


class GoalProgress(object):
    """Calculate goal progress without reading ROS or sending car commands."""

    def __init__(self, tolerance=1.0, approach_distance=5.0,
                 deceleration=0.5, reaction_time=0.5,
                 stopped_speed=0.1, stopped_duration=1.0):
        values = [tolerance, approach_distance, deceleration, reaction_time,
                  stopped_speed, stopped_duration]
        if (not all(finite(v) for v in values) or tolerance <= 0 or
                approach_distance < tolerance or deceleration <= 0 or
                reaction_time < 0 or stopped_speed < 0 or stopped_duration <= 0):
            raise ValueError("Invalid goal monitor settings")
        self.tolerance = tolerance
        self.approach_distance = approach_distance
        self.deceleration = deceleration
        self.reaction_time = reaction_time
        self.stopped_speed = stopped_speed
        self.stopped_duration = stopped_duration
        self.reset()

    def reset(self):
        self.state = "DRIVING"
        self.stopped_since = None

    def unavailable(self):
        # Missing readings do not prove that the car stayed stopped during that time.
        self.stopped_since = None

    def update(self, distance, speed, now):
        if not all(finite(v) for v in [distance, speed, now]) or distance < 0:
            raise ValueError("Invalid goal observation")
        speed = abs(speed)
        approach = max(self.approach_distance,
                       self.tolerance + speed * self.reaction_time +
                       speed * speed / (2.0 * self.deceleration))
        # Small position changes must not restart driving after a stop is indicated.
        if distance <= self.tolerance:
            if self.state != "STOPPED":
                self.state = "STOPPING"
        elif distance <= approach and self.state == "DRIVING":
            self.state = "APPROACHING"
        if distance <= self.tolerance and speed <= self.stopped_speed:
            if self.stopped_since is None:
                self.stopped_since = now
            if now - self.stopped_since >= self.stopped_duration:
                self.state = "STOPPED"
        else:
            self.stopped_since = None
            if self.state == "STOPPED":
                self.state = "STOPPING"
        stopping = self.state in ("APPROACHING", "STOPPING", "STOPPED")
        speed_limit = None
        if self.state in ("STOPPING", "STOPPED"):
            speed_limit = 0.0
        elif self.state == "APPROACHING":
            speed_limit = math.sqrt(2.0 * self.deceleration *
                                    max(0.0, distance - self.tolerance))
        return {"progress_state": self.state,
                "would_request_stop": stopping,
                "suggested_speed_limit_mps": speed_limit,
                "approach_distance_m": approach}


class GoalSpeedProfile(object):
    """Gradually lower the speed limit; remove it only after a new goal passes the input checks."""

    def __init__(self, deceleration):
        if not finite(deceleration) or deceleration <= 0:
            raise ValueError("Goal deceleration must be positive")
        self.deceleration = deceleration
        self.reset()

    def reset(self):
        self.limit = None
        self.target = None
        self.last_time = None

    def update(self, requested_limit, actual_speed, now):
        if not finite(now):
            raise ValueError("Invalid profile time")
        if requested_limit is not None:
            if (not finite(requested_limit) or requested_limit < 0 or
                    actual_speed is None or not finite(actual_speed)):
                raise ValueError("Invalid goal speed limit or speed")
            if self.limit is None:
                # Start at actual speed, even if the first request is zero.
                self.limit = abs(actual_speed)
                self.target = min(self.limit, requested_limit)
                self.last_time = now
            else:
                self.target = min(self.target, requested_limit)
        if self.limit is not None:
            # Avoid a large single target step if the process was paused.
            elapsed = min(0.5, max(0.0, now - self.last_time))
            self.limit = max(self.target, self.limit - self.deceleration * elapsed)
            self.last_time = now
        return self.limit


class PhysicalGoalStopMonitor(object):
    def __init__(self):
        import rospy
        import tf
        from geometry_msgs.msg import PoseStamped, TwistStamped
        from std_msgs.msg import String
        self.rospy, self.tf, self.String = rospy, tf, String
        self.lock = threading.RLock()
        self.listener = tf.TransformListener()
        self.frame = rospy.get_param("~reference_frame", "map").lstrip("/")
        self.input_timeout = float(rospy.get_param("~input_timeout_s", 2.0))
        self.ad_timeout = float(rospy.get_param("~ad_status_timeout_s", 5.0))
        self.rate = float(rospy.get_param("~publish_rate_hz", 10.0))
        self.replay = bool(rospy.get_param("~replay_mode", False))
        self.control_enabled = bool(rospy.get_param("~enable_speed_control", False)) and not self.replay
        if (not self.frame or not all(finite(v) and v > 0 for v in
                                     [self.input_timeout, self.ad_timeout, self.rate])):
            raise ValueError("Invalid monitor frame, timeouts or rate")
        self.progress = GoalProgress(
            float(rospy.get_param("~goal_tolerance_m", 1.0)),
            float(rospy.get_param("~approach_distance_m", 5.0)),
            float(rospy.get_param("~suggested_deceleration_mps2", 0.5)),
            float(rospy.get_param("~reaction_time_s", 0.5)),
            float(rospy.get_param("~stopped_speed_mps", 0.1)),
            float(rospy.get_param("~stopped_duration_s", 1.0)))
        self.speed_profile = GoalSpeedProfile(self.progress.deceleration)
        self.goal = self.pose = self.speed = self.ad = None
        self.pose_received = self.speed_received = self.ad_received = None
        self.goal_id = 0
        self.recovery_hold = False
        self.new_goal_pending = False
        self.goal_error = self.pose_error = self.speed_error = self.ad_error = None
        self.pub = rospy.Publisher("/adeye/goal_stop_status", String,
                                   queue_size=1, latch=True)
        self.limit_pub = None
        if self.control_enabled:
            from std_msgs.msg import Float64
            self.Float64 = Float64
            self.limit_pub = rospy.Publisher("/adeye/goal_stop_speed_limit", Float64,
                                             queue_size=1, latch=True)
        self.subs = [
            rospy.Subscriber(rospy.get_param("~goal_topic", "/move_base_simple/goal"),
                             PoseStamped, self.goal_callback, queue_size=1),
            rospy.Subscriber(rospy.get_param("~pose_topic", "/current_pose"),
                             PoseStamped, self.pose_callback, queue_size=1),
            rospy.Subscriber(rospy.get_param("~speed_topic", "/current_velocity"),
                             TwistStamped, self.speed_callback, queue_size=1),
            rospy.Subscriber(rospy.get_param("~ad_status_topic", "/adeye/vehicle_fault_status"),
                             String, self.ad_callback, queue_size=1)]
        if rospy.get_param("/adeye/enable_manual_recovery", False):
            self.recovery_token = rospy.get_param("/adeye/vehicle_recovery_token", "")
            self.recovery_hold = bool(self.recovery_token)
            self.recovery_ack = rospy.Publisher("/adeye/vehicle_recovery_ack", String, queue_size=10, latch=True)
            self.recovery_sub = rospy.Subscriber("/adeye/vehicle_recovery_control", String, self.recovery_callback, queue_size=10)
        self.shutdown = threading.Event()
        rospy.on_shutdown(self.shutdown.set)
        self.worker = threading.Thread(target=self.run)
        self.worker.daemon = True
        self.worker.start()
        rospy.loginfo("Goal arrival: %s", "live target-speed limiting enabled; no brake hold"
                      if self.control_enabled else "monitor only")

    @staticmethod
    def valid_pose(message):
        p, q = message.pose.position, message.pose.orientation
        return (bool(message.header.frame_id.strip("/")) and
                all(finite(v) for v in [p.x, p.y, p.z, q.x, q.y, q.z, q.w]) and
                q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w > 1e-12)

    def goal_callback(self, message):
        with self.lock:
            if getattr(self, "recovery_hold", False):
                return
            if not self.valid_pose(message):
                self.goal_error = "INVALID_GOAL"
                return
            self.goal_error = None
            self.goal = copy.deepcopy(message)
            self.goal_id += 1
            self.progress.reset()
            self.new_goal_pending = True

    def recovery_callback(self, message):
        action, separator, token = message.data.partition(":")
        if not separator or not token or any(c not in "0123456789abcdef" for c in token):
            return
        with self.lock:
            if action == "PREPARE":
                self.recovery_hold = True
                self.recovery_token = token
                self.goal = None
                self.goal_error = None
                self.new_goal_pending = False
                self.progress.reset()
                self.speed_profile.reset()
                if self.limit_pub is not None:
                    self.limit_pub.publish(self.Float64(data=-1.0))
                phase = "prepared"
            elif action == "RELEASE" and token == self.recovery_token:
                self.recovery_hold = False
                phase = "released"
            else:
                return
            self.recovery_ack.publish(self.String(data=json.dumps({"component": self.rospy.get_name(), "token": token, "phase": phase})))

    def pose_callback(self, message):
        with self.lock:
            if not self.valid_pose(message):
                self.pose_error = "INVALID_POSE"
                return
            self.pose_error = None
            self.pose, self.pose_received = copy.deepcopy(message), time.time()

    def speed_callback(self, message):
        with self.lock:
            if not finite(message.twist.linear.x):
                self.speed_error = "INVALID_SPEED"
                return
            self.speed_error = None
            self.speed, self.speed_received = copy.deepcopy(message), time.time()

    def ad_callback(self, message):
        with self.lock:
            try:
                status = json.loads(message.data)
                if not isinstance(status, dict):
                    raise ValueError("AD status must be an object")
                stamp = float(status["wall_time_s"])
                if status.get("state") not in ("INITIALIZING", "ENABLED", "ENGAGED", "FAULT") or not finite(stamp):
                    raise ValueError("Invalid AD state")
                status["wall_time_s"] = stamp
                self.ad, self.ad_received = status, time.time()
                self.ad_error = None
            except (ValueError, TypeError, KeyError):
                self.ad_error = "INVALID_AD_STATUS"

    def transformed(self, message):
        pose = copy.deepcopy(message)
        pose.header.frame_id = pose.header.frame_id.lstrip("/")
        if pose.header.frame_id != self.frame:
            # Put the car and goal positions in the same coordinate system before comparing them.
            pose.header.stamp = self.rospy.Time(0)
            pose = self.listener.transformPose(self.frame, pose)
        return pose.pose.position

    def timed_input_reason(self, message, received, now, name):
        if message is None:
            return "WAITING_FOR_" + name
        if now - received > self.input_timeout:
            return name + "_OUTDATED"
        stamp = message.header.stamp.to_sec()
        age = self.rospy.Time.now().to_sec() - stamp
        if stamp <= 0 or age > self.input_timeout or age < -self.input_timeout:
            return name + "_TIMESTAMP_INVALID_OR_OUTDATED"
        return None

    def status(self):
        now = time.time()
        result = {"monitor_only": not self.control_enabled,
                  "actuator_commands_enabled": self.control_enabled,
                  "reference_frame": self.frame, "goal_id": self.goal_id,
                  "state": "WAITING_FOR_GOAL", "reason": "WAITING_FOR_GOAL",
                  "progress_state": self.progress.state if self.goal else "WAITING_FOR_GOAL",
                  "would_request_stop": False, "suggested_speed_limit_mps": None,
                  "distance_to_goal_m": None, "speed_mps": None,
                  "goal": None, "ad_state": self.ad.get("state") if self.ad else None,
                  "ad_status_age_s": now - self.ad["wall_time_s"] if self.ad else None,
                  "wall_time_s": now, "distance_method": "straight_line_2d",
                  "limitations": "No route acceptance check; AD status is event-based; speed timestamp does not verify CAN freshness"}
        issues = []
        warnings = []
        if self.replay:
            issues.append("REPLAY_MODE_DISABLED")
        if self.goal_error:
            issues.append(self.goal_error)
        if self.goal is None:
            issues.append("WAITING_FOR_GOAL")
        for message, received, name in [(self.pose, self.pose_received, "POSE"),
                                        (self.speed, self.speed_received, "SPEED")]:
            reason = self.timed_input_reason(message, received, now, name)
            if reason:
                if message is None:
                    issues.append(reason)
                else:
                    warnings.append(reason)
        issues.extend(error for error in [self.pose_error, self.speed_error, self.ad_error] if error)
        if self.ad is None:
            issues.append("WAITING_FOR_AD_STATUS")
        elif (now - self.ad_received > self.ad_timeout or
              not 0 <= now - self.ad["wall_time_s"] <= self.ad_timeout):
            warnings.append("AD_STATUS_OUTDATED")
        if self.ad is not None and self.ad["state"] != "ENGAGED":
            issues.append("AD_NOT_ENGAGED")
        if self.speed is not None:
            result["speed_mps"] = self.speed.twist.linear.x
        if self.goal is not None and self.pose is not None:
            try:
                goal, pose = self.transformed(self.goal), self.transformed(self.pose)
                result["goal"] = {"x": goal.x, "y": goal.y, "z": goal.z}
                result["distance_to_goal_m"] = math.hypot(goal.x - pose.x, goal.y - pose.y)
                if not finite(result["distance_to_goal_m"]):
                    issues.append("INVALID_DISTANCE")
                    result["distance_to_goal_m"] = None
            except (self.tf.LookupException, self.tf.ConnectivityException,
                    self.tf.ExtrapolationException):
                issues.append("TF_UNAVAILABLE")
        if issues:
            self.progress.unavailable()
            result["state"] = "WAITING_FOR_GOAL" if self.goal is None else "INPUT_UNAVAILABLE"
            result["reason"] = issues[0]
        else:
            result.update(self.progress.update(result["distance_to_goal_m"],
                                               result["speed_mps"], now))
            result["state"] = self.progress.state
            result["reason"] = "MONITOR_ONLY_" + self.progress.state
        result["input_issues"] = issues
        result["freshness_warnings"] = warnings
        result["freshness_checks_informational_only"] = True
        result["uses_last_known_inputs"] = bool(warnings)
        return result

    def control_limit(self, status):
        release = False
        if self.new_goal_pending and not status["input_issues"]:
            self.new_goal_pending = False
            release = self.control_enabled and self.speed_profile.limit is not None
            self.speed_profile.reset()
        requested = None
        if not release and self.control_enabled and status["would_request_stop"]:
            requested = status["suggested_speed_limit_mps"]
        # A missing message never creates a stop request or releases an existing one.
        limit = self.speed_profile.update(requested, status["speed_mps"], time.time())
        status["goal_stop_speed_limit_mps"] = limit
        status["goal_stop_requested_limit_mps"] = self.speed_profile.target
        status["goal_stop_latched"] = limit is not None
        status["new_goal_release_implemented"] = True
        status["new_goal_pending"] = self.new_goal_pending
        status["released_for_new_goal"] = release
        if release:
            status["reason"] = "NEW_GOAL_RELEASED"
        status["goal_acceptance"] = "valid RViz goal and available inputs; not planner route acceptance"
        status["brake_hold_requested"] = False
        # -1 removes the speed limit on this topic. Other negative values are not allowed.
        # First send the request to remove the old limit. Check the new goal's limit on the next update.
        return -1.0 if release else limit

    def run(self):
        while not self.shutdown.is_set() and not self.rospy.is_shutdown():
            with self.lock:
                message = self.status()
                limit = self.control_limit(message)
                # Use the last known AD state; its age is only a warning.
                publish_limit = (not getattr(self, "recovery_hold", False) and self.control_enabled and limit is not None and
                                 self.ad is not None and self.ad["state"] == "ENGAGED")
                if publish_limit:
                    self.limit_pub.publish(self.Float64(data=limit))
            self.pub.publish(self.String(data=json.dumps(message, allow_nan=False)))
            self.shutdown.wait(1.0 / self.rate)


if __name__ == "__main__":
    import rospy
    rospy.init_node("physical_goal_stop_monitor")
    PhysicalGoalStopMonitor()
    rospy.spin()
