#!/usr/bin/env python
"""Show known configuration details and optionally manage fault switches for this session."""
import hashlib
import json
import math
import os
import sys
import threading
import time
import uuid
import copy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from test_feature_registry import get_registry, validate_registry
from configuration_diagnostics import diagnose

REGISTRY_TOPIC = "/adeye/test_configuration_registry"
STATUS_TOPIC = "/adeye/test_configuration_status"
OBSERVED_TOPICS = ("/steering_requested_phy", "/acceleration_requested_phy", "/points_raw",
                   "/adeye/fault_injection/fix", "/adeye/goal_stop_status", "/adeye/physical_fault_status")
try:
    string_types = (basestring,)
except NameError:
    string_types = (str,)


def read_fault_report(text):
    data = json.loads(text)
    if not isinstance(data, dict) or not all(isinstance(data.get(key), string_types)
                                           and data[key] for key in ("source", "event", "fault")):
        raise ValueError("invalid fault status envelope")
    result = {key: data[key][:200] for key in ("source", "event", "fault")}
    for key in ("value", "timeout_s", "min_value", "max_value"):
        value = data.get(key)
        if value is not None:
            if isinstance(value, bool) or not isinstance(value, (int, float)) or math.isnan(value) or math.isinf(value):
                raise ValueError("invalid fault status number")
            result[key] = value
    if "active" in data:
        if not isinstance(data["active"], bool):
            raise ValueError("invalid active flag")
        result["active"] = data["active"]
    detail = data.get("detail", "")
    result["detail"] = (detail if isinstance(detail, string_types) else str(detail))[:1000]
    return result


def graph_evidence(system_state):
    if not isinstance(system_state, (tuple, list)) or len(system_state) != 3:
        raise ValueError("invalid ROS master graph")
    nodes, routes = set(), {}
    for section in system_state:
        for resource, names in section:
            nodes.update(names)
    for topic, names in system_state[0]:
        if topic in OBSERVED_TOPICS:
            routes[topic] = sorted(set(names))
    return sorted(nodes), routes


def make_status(registry, boot_id, registry_id, profile, startup_request, system_state,
                manager_uri=None, graph_error=None, reports=(), now=None, roslaunch_run_id=""):
    now = time.time() if now is None else now
    nodes, routes = ([], {}) if system_state is None else graph_evidence(system_state)
    # Launch arguments show what was requested, not what each running node is using.
    observations = [{"id": feature["id"], "launch_requested": startup_request.get(feature["id"]),
                     "applied_state": "unknown", "applied_value": None,
                     "reason": "No component-confirmed configuration/boot acknowledgement exists yet."}
                    for feature in registry["features"]]
    return {
        "schema_version": 1, "phase": "read_only", "read_only": True,
        "monitor_boot_id": boot_id, "registry_id": registry_id,
        "sequence": 0, "wall_time_s": now, "profile": profile,
        "roslaunch_run_id": roslaunch_run_id,
        "manager": {"registered": None if system_state is None else "/manager" in nodes,
                    "api_uri": manager_uri, "confirmed_boot_id": None,
                    "identity_kind": "ROS registration only; not a confirmed manager boot"},
        "baseline": {"restore_ready": False, "state": "launch_request_only",
                     "reason": "Resolved common-launch requests are captured, but component boot settings and override ownership are not confirmed."},
        "graph": {"available": system_state is not None, "error": graph_error,
                  "registered_nodes": nodes, "publishers": routes,
                  "meaning": "Master registration is not node health, command acceptance, or proof of applied configuration."},
        "features": observations, "recent_fault_reports": list(reports),
        "actions": {"apply_available": False, "reset_available": False, "save_available": False},
        "warning": "Read-only evidence, not a verified reset baseline. Input ages are informational and cause no vehicle action.",
    }


class TestConfigurationMonitor(object):
    def __init__(self):
        import rospy
        import rosgraph
        from std_msgs.msg import String
        self.rospy, self.String = rospy, String
        self.registry = get_registry()
        self.availability_enabled = bool(rospy.get_param("/adeye/enable_fault_availability", False))
        if self.availability_enabled:
            for feature in self.registry["features"]:
                if feature["kind"] == "fault_availability" or feature["id"] in ("steering_fault_processing", "acceleration_fault_processing", "lidar_fault_processing", "gnss_fault_processing"):
                    feature["current_change_mode"] = "runtime_idle_only"
                    feature["notes"] = "Availability is enforced by the owning proxy. Stop its test and cancel affected pending schedules before changing it."
                    if feature["kind"] != "fault_availability":
                        feature["label"] = {"steering_fault_processing": "Steering fault processing",
                            "acceleration_fault_processing": "Acceleration fault processing",
                            "lidar_fault_processing": "LiDAR fault processing",
                            "gnss_fault_processing": "GNSS fault processing"}[feature["id"]]
                        feature["disabled_behavior"] = "Reject new faults in this group; keep normal input forwarding and individual fault settings unchanged. Does not start a missing proxy or sensor path."
                        feature["notes"] += " On allows individually enabled faults; Off does not bypass or remove the proxy itself."
        validate_registry(self.registry)
        self.boot_id = uuid.uuid4().hex
        self.registry_id = hashlib.sha256(json.dumps(self.registry, sort_keys=True).encode("utf-8")).hexdigest()
        self.profile = str(rospy.get_param("~profile", "unspecified"))
        self.startup_request = rospy.get_param("~startup_request", {})
        if not isinstance(self.startup_request, dict) or any(not isinstance(value, bool) for value in self.startup_request.values()):
            raise ValueError("startup_request must contain launch-resolved booleans")
        self.run_id = str(rospy.get_param("/run_id", ""))
        self.rate = float(rospy.get_param("~publish_rate_hz", 1.0))
        if not 0 < self.rate <= 5:
            raise ValueError("configuration monitor rate must be >0 and <=5 Hz")
        self.master = rosgraph.Master(rospy.get_name())
        self.availability = None
        self.startup = None
        self.control_errors = {}
        self.last_availability = None
        if self.availability_enabled:
            from fault_availability_coordinator import AvailabilityCoordinator
            self.availability = AvailabilityCoordinator(rospy, String, self.registry, self.boot_id)
            try:
                from startup_configuration import StartupStore, StartupServices
                launch_settings = rospy.get_param("~startup_settings", None) if rospy.get_param("~startup_configuration_enabled", False) else None
                self.startup = StartupServices(rospy, self.boot_id, StartupStore(launch_settings=launch_settings))
            except Exception as error:
                self.control_errors["startup"] = str(error)
        self.lock = threading.Lock()
        self.reports = {}
        self.clock = getattr(time, "monotonic", time.time)
        self.registry_pub = rospy.Publisher(REGISTRY_TOPIC, String, queue_size=1, latch=True)
        self.pub = rospy.Publisher(STATUS_TOPIC, String, queue_size=1, latch=True)
        self.sub = rospy.Subscriber("/adeye/physical_fault_status", String, self.fault_callback, queue_size=20)
        self.registry_pub.publish(String(data=json.dumps({"schema_version": 1, "monitor_boot_id": self.boot_id,
            "registry_id": self.registry_id, "registry": self.registry}, allow_nan=False)))
        self.shutdown = threading.Event()
        rospy.on_shutdown(self.shutdown.set)
        self.worker = threading.Thread(target=self.run)
        self.worker.daemon = True
        self.worker.start()
        rospy.loginfo("Test Configuration: %s", "session-only fault availability enabled" if self.availability else "read-only")

    def fault_callback(self, message):
        try:
            report = read_fault_report(message.data)
        except (ValueError, TypeError, KeyError, AttributeError):
            return
        if self.availability: self.availability.fault_report(report)
        with self.lock:
            key = (report["source"], report["fault"])
            self.reports[key] = (self.clock(), report)
            if len(self.reports) > 64:
                del self.reports[min(self.reports, key=lambda item: self.reports[item][0])]

    def sample(self):
        system_state, manager_uri, error = None, None, None
        try:
            system_state = self.master.getSystemState()
            # If the manager address cannot be read, still show the listed ROS connections.
            if "/manager" in graph_evidence(system_state)[0]:
                try:
                    manager_uri = self.master.lookupNode("/manager")
                except Exception as lookup_error:
                    error = "Manager URI lookup failed: " + str(lookup_error)
        except Exception as graph_error:
            system_state = None
            error = "ROS master graph unavailable: " + str(graph_error)
        now = self.clock()
        with self.lock:
            reports = [dict(report, received_age_s=max(0.0, now - received),
                            evidence="reported fault event, not an availability/configuration acknowledgement")
                       for received, report in self.reports.values()]
        result = make_status(self.registry, self.boot_id, self.registry_id, self.profile,
                           self.startup_request, system_state, manager_uri, error,
                           sorted(reports, key=lambda report: (report["source"], report["fault"])),
                           roslaunch_run_id=self.run_id)
        errors = dict(self.control_errors)
        availability = None
        if self.availability:
            try:
                availability = self.availability.status()
                self.last_availability = copy.deepcopy(availability)
            except Exception as error:
                errors["availability"] = str(error)
                availability = copy.deepcopy(self.last_availability)
                if availability is not None: availability["read_error"] = str(error)
        if availability is not None:
            result.update(phase="fault_availability", read_only=False, availability=availability)
            result["actions"].update(apply_available=bool(availability["commands"]), reset_available=availability["owned_override_count"] > 0)
            result["warning"] = "Individual and group fault availability are editable. Other settings and the whole-manager reset baseline remain unconfirmed."
            for feature in result["features"]:
                metadata = next(item for item in self.registry["features"] if item["id"] == feature["id"])
                entry = availability["commands"].get(metadata.get("command")) or availability["groups"].get(feature["id"])
                if entry and "availability" not in errors and entry["component"] in result["graph"]["registered_nodes"]:
                    feature.update(applied_state="on" if entry["enabled"] else "off", applied_value=entry["enabled"],
                        reason="Availability confirmed by %s (boot %s, revision %s)" % (entry["component"], entry["boot_id"], entry["revision"]))
                elif entry:
                    feature["reason"] = "Stored acknowledgement exists, but current configuration is unknown: observer/graph unavailable or component not registered. No vehicle action requested."
            if "availability" in errors:
                result["actions"].update(apply_available=False, reset_available=False)
        if self.startup and availability is not None:
            try: result["startup"] = self.startup.status()
            except Exception as error:
                errors["startup"] = str(error)
                # If startup controls cannot be set up, keep reporting the runtime fault switches.
                try:
                    result["startup"] = self.startup.store.status()
                    result["startup"].update(services_available=False,
                        service_root="/adeye/test_configuration/session/%s/startup/r%s" % (self.boot_id, result["startup"]["revision"]))
                except Exception as store_error: errors["startup"] += "; " + str(store_error)
            if "startup" in result:
                result["actions"]["save_available"] = "startup" not in errors and not result["startup"]["disk_error"] and not result["startup"]["validation_errors"]
        result["diagnostics"] = diagnose(system_state, result.get("startup"), availability, errors)
        return result

    def run(self):
        sequence = 0
        previous = None
        while not self.shutdown.is_set() and not self.rospy.is_shutdown():
            try:
                result = self.sample()
                previous = copy.deepcopy(result)
            except Exception as error:
                # Keep reporting the failure, without issuing any control request.
                result = copy.deepcopy(previous) if previous is not None else make_status(
                    self.registry, self.boot_id, self.registry_id, self.profile, self.startup_request, None)
                result["wall_time_s"] = time.time()
                for feature in result["features"]:
                    feature.update(applied_state="unknown", applied_value=None, reason="Observer failed; previous state is not current confirmation")
                result["actions"] = dict(apply_available=False, reset_available=False, save_available=False)
                if result.get("availability") is not None: result["availability"]["read_error"] = str(error)
                if result.get("startup") is not None: result["startup"]["services_available"] = False
                result["diagnostics"] = diagnose(None, errors={"observer": str(error)})
            sequence += 1
            result["sequence"] = sequence
            self.pub.publish(self.String(data=json.dumps(result, allow_nan=False)))
            self.shutdown.wait(1.0 / self.rate)


if __name__ == "__main__":
    import rospy
    rospy.init_node("test_configuration_monitor")
    TestConfigurationMonitor()
    rospy.spin()
