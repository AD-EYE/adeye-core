#!/usr/bin/env python
"""Show whether a manual reset may be possible. Never reset anything or send car commands."""
import json
import math
import threading
import time


# Only consider reset reasons whose existing car-handling code is understood.
# Steering intervention is shown, but we have not traced how it triggers emergency mode yet.
DRIVER_INTERVENTION_REASONS = frozenset([
    "DRIVER_BRAKE_INTERVENTION", "DRIVER_ACCELERATOR_INTERVENTION"])
UNVERIFIED_STEPS = [
    "AUTOMATIC_RECOVERY_NOT_IMPLEMENTED", "MANUAL_RESET_REQUIRES_OPERATOR_CONFIRMATION",
    "VEHICLE_RESET_SEQUENCE_UNVERIFIED", "CAN_SAMPLE_FRESHNESS_UNVERIFIED",
    "OEM_FAULT_CLEARANCE_UNVERIFIED"]


def finite_number(value):
    return (not isinstance(value, bool) and isinstance(value, (int, float)) and
            not math.isnan(value) and not math.isinf(value))


def classify_fault(reason):
    if reason in DRIVER_INTERVENTION_REASONS:
        return "DRIVER_INTERVENTION_CANDIDATE"
    if reason == "CAN_SIGNAL_INVALID":
        return "CAN_COMMUNICATION_FAULT"
    if reason.startswith("WHEEL_LOCK_"):
        return "WHEEL_LOCK_FAULT"
    return "UNSUPPORTED_OR_UNKNOWN_FAULT"


def read_snapshot(data):
    value = json.loads(data)
    if not isinstance(value, dict) or value.get("schema_version") != 1 or value.get("source") != "ros2can":
        raise ValueError("Unsupported recovery-input message")
    if value.get("state") not in ("INITIALIZING", "ENABLED", "ENGAGED", "FAULT"):
        raise ValueError("Unknown software state")
    for name in ["reason", "detail", "fault_entry_reason", "fault_entry_detail"]:
        if not isinstance(value.get(name), type(u"")) and not isinstance(value.get(name), str):
            raise ValueError("Missing or invalid " + name)
    for name in ["speed_mps", "brake_interrupt", "accelerator_interrupt", "steering_interrupt",
                 "ad_activation_ok", "ad_mode_state"]:
        if name not in value or (value[name] is not None and not finite_number(value[name])):
            raise ValueError("Missing or invalid " + name)
    if not finite_number(value.get("wall_time_s")):
        raise ValueError("Invalid source time")
    return value


def assess_recovery(snapshot, input_age_s=None, now=None, input_timeout_s=2.0,
                    stopped_speed_mps=0.1):
    """Show information only. This result does not authorize a reset."""
    result = {
        "schema_version": 1, "assessment_only": True,
        "scope": "automatic recovery assessment only; manual software reset is a separate opt-in operation",
        "manual_reset_service": "/adeye/recover_to_ready",
        "manual_reset_status_topic": "/adeye/vehicle_recovery_operation_status",
        "recovery_actions_implemented": False, "recovery_enabled": False,
        "recovery_command_available": False,
        "eligible_for_recovery": False, "vehicle_fault_confirmed": None,
        "can_sample_freshness_verified": False,
        "freshness_checks_informational_only": True,
        "assessment_state": "WAITING_FOR_INPUT", "software_fault_latched": None,
        "candidate_for_future_recovery": False, "observed_prerequisites_met": None,
        "fault_category": "UNKNOWN", "fault_entry_category": "UNKNOWN",
        "reason": None, "detail": None, "fault_entry_reason": None,
        "fault_entry_detail": None, "software_state": None,
        "observations": {}, "observed_conditions": {},
        "blockers": [], "unverified_steps": list(UNVERIFIED_STEPS),
        "freshness_warnings": [], "input_age_s": input_age_s,
        "standstill_threshold_mps": stopped_speed_mps,
        "wall_time_s": time.time() if now is None else now,
        "limitations": "CAN readings may be cached or initial values; OEM emergency clearance is unverified. This monitor does not approve or execute resets. Manual software reset requires operator confirmation.",
    }
    if snapshot is None:
        result["blockers"] = ["WAITING_FOR_RECOVERY_INPUTS"]
        return result
    warnings = result["freshness_warnings"]
    if input_age_s is not None and (input_age_s < 0 or input_age_s > input_timeout_s):
        warnings.append("RECOVERY_INPUTS_OUTDATED")
    source_age = result["wall_time_s"] - snapshot["wall_time_s"]
    result["source_snapshot_age_s"] = source_age
    if source_age < 0 or source_age > input_timeout_s:
        warnings.append("SOURCE_TIME_OUTDATED_OR_CLOCK_DIFFERENT")
    result["software_state"] = snapshot["state"]
    result["software_fault_latched"] = snapshot["state"] == "FAULT"
    for field in ["reason", "detail", "fault_entry_reason", "fault_entry_detail"]:
        result[field] = snapshot[field]
    result["observations"] = {name: snapshot[name] for name in [
        "speed_mps", "brake_interrupt", "accelerator_interrupt", "steering_interrupt",
        "ad_activation_ok", "ad_mode_state"]}
    speed = snapshot["speed_mps"]
    conditions = result["observed_conditions"]
    conditions["stationary_from_cached_speed"] = None if speed is None else abs(speed) <= stopped_speed_mps
    conditions["ad_disengaged_from_cached_signal"] = (
        snapshot["ad_mode_state"] == 0 if snapshot["ad_mode_state"] in (0, 1) else None)
    for field in ["brake_interrupt", "accelerator_interrupt", "steering_interrupt"]:
        conditions[field + "_clear_from_cached_signal"] = snapshot[field] == 0 if snapshot[field] in (0, 1) else None
    conditions["vehicle_allows_ad_from_cached_signal"] = (
        snapshot["ad_activation_ok"] == 1 if snapshot["ad_activation_ok"] in (0, 1) else None)
    observed_blockers = []
    for name, met in sorted(conditions.items()):
        if met is not True:
            observed_blockers.append(name.upper() + ("_UNKNOWN" if met is None else "_NOT_MET"))
    result["observed_prerequisites_met"] = not observed_blockers
    if not result["software_fault_latched"]:
        result["fault_category"] = result["fault_entry_category"] = "NO_SOFTWARE_FAULT"
        result["assessment_state"] = "NO_SOFTWARE_FAULT"
        return result
    result["fault_category"] = classify_fault(snapshot["reason"])
    result["fault_entry_category"] = classify_fault(snapshot["fault_entry_reason"])
    candidate = (snapshot["reason"] in DRIVER_INTERVENTION_REASONS and
                 snapshot["fault_entry_reason"] in DRIVER_INTERVENTION_REASONS)
    result["candidate_for_future_recovery"] = candidate
    if not candidate:
        observed_blockers.insert(0, "FAULT_REASON_NOT_SUPPORTED_FOR_RECOVERY")
    result["assessment_state"] = "CANDIDATE_UNVERIFIED" if candidate and not observed_blockers else "BLOCKED"
    result["blockers"] = observed_blockers + list(UNVERIFIED_STEPS)
    return result


class VehicleRecoveryMonitor(object):
    def __init__(self):
        import rospy
        from std_msgs.msg import String
        self.rospy, self.String = rospy, String
        self.lock = threading.Lock()
        self.snapshot = self.received = self.input_error = None
        self.input_timeout = float(rospy.get_param("~input_timeout_s", 2.0))
        self.stopped_speed = float(rospy.get_param("~stopped_speed_mps", 0.1))
        self.rate = float(rospy.get_param("~publish_rate_hz", 1.0))
        if not all(finite_number(v) and v > 0 for v in [self.input_timeout, self.stopped_speed, self.rate]):
            raise ValueError("Invalid recovery monitor settings")
        self.pub = rospy.Publisher("/adeye/vehicle_recovery_status", String, queue_size=1, latch=True)
        self.sub = rospy.Subscriber("/adeye/vehicle_recovery_inputs", String, self.callback, queue_size=1)
        self.shutdown = threading.Event()
        rospy.on_shutdown(self.shutdown.set)
        self.worker = threading.Thread(target=self.run)
        self.worker.daemon = True
        self.worker.start()
        rospy.loginfo("Vehicle recovery assessment only; no reset or vehicle commands are available")

    def callback(self, message):
        with self.lock:
            try:
                self.snapshot = read_snapshot(message.data)
                self.received = time.time()
                self.input_error = None
            except (ValueError, TypeError, KeyError, AttributeError) as error:
                self.snapshot = self.received = None
                self.input_error = str(error)

    def status(self):
        now = time.time()
        with self.lock:
            result = assess_recovery(self.snapshot,
                                     None if self.received is None else now - self.received,
                                     now, self.input_timeout, self.stopped_speed)
            if self.input_error:
                result["assessment_state"] = "INPUT_INVALID"
                result["blockers"] = ["INVALID_RECOVERY_INPUTS"]
                result["input_error"] = self.input_error
            return result

    def run(self):
        while not self.shutdown.is_set() and not self.rospy.is_shutdown():
            self.pub.publish(self.String(data=json.dumps(self.status(), allow_nan=False)))
            self.shutdown.wait(1.0 / self.rate)


if __name__ == "__main__":
    import rospy
    rospy.init_node("vehicle_recovery_monitor")
    VehicleRecoveryMonitor()
    rospy.spin()
