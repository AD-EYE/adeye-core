"""Control which sensor faults can start. Does not start faults or change sensor topics."""
import json
import uuid
try:
    string_types = (basestring,)
except NameError:
    string_types = (str,)


class SensorAvailability(object):
    def __init__(self, node, commands):
        import rospy
        from std_msgs.msg import String
        self.rospy, self.String, self.node = rospy, String, node
        self.component, self.boot = rospy.get_name(), uuid.uuid4().hex
        self.entries = {}
        self.groups = {}
        for command in commands:
            baseline = rospy.get_param("~fault_availability/" + command, True)
            if not isinstance(baseline, bool): raise ValueError("fault availability baseline must be boolean")
            self.entries[command] = dict(baseline=baseline, enabled=baseline, revision=0, owner="")
            prefix = "LIDAR" if command.startswith("LIDAR") else "GNSS"
            self.groups[command] = prefix + "_GROUP_command"
        for group in set(self.groups.values()):
            baseline = rospy.get_param("~enable_" + group.split("_")[0].lower() + "_faults", True)
            if not isinstance(baseline, bool): raise ValueError("group baseline must be boolean")
            self.entries[group] = dict(baseline=baseline, enabled=baseline, revision=0, owner="")
        self.pub = rospy.Publisher("/adeye/fault_availability_state", String, queue_size=10, latch=True)
        self.sub = rospy.Subscriber("/adeye/fault_availability_control", String, self.control, queue_size=20)
        self.publish()

    def publish(self, request="", accepted=False, detail="snapshot"):
        self.pub.publish(self.String(data=json.dumps(dict(schema_version=1, component=self.component,
            boot_id=self.boot, busy=bool(self.node.active_fault or self.node.recovery_hold), commands=self.entries,
            request_id=request, accepted=accepted, detail=detail))))

    def control(self, message):
        try:
            data = json.loads(message.data)
            if data.get("schema_version") != 1 or data.get("component") != self.component: return
            request, owner = data.get("request_id", ""), data.get("owner", "")
            if not all(isinstance(value, string_types) and 0 < len(value) <= 64 and all(c in "0123456789abcdef" for c in value) for value in (request, owner)): return
            with self.node._lock:
                entry = self.entries.get(data.get("fault"))
                reason = None
                if entry is None: reason = "unsupported fault"
                elif data.get("boot_id") != self.boot or data.get("revision") != entry["revision"]: reason = "component restarted or availability revision changed"
                elif self.node.active_fault or self.node.recovery_hold: reason = "controller busy; stop its test or recovery first"
                elif entry["owner"] not in ("", owner): reason = "override belongs to another configuration session"
                elif data.get("operation") == "reset":
                    entry["enabled"], entry["owner"] = entry["baseline"], ""
                elif data.get("operation") == "set" and isinstance(data.get("value"), bool):
                    entry["enabled"] = data["value"]
                    entry["owner"] = "" if data["value"] == entry["baseline"] else owner
                else: reason = "invalid availability request"
                if reason is None: entry["revision"] += 1
                self.publish(request, reason is None, reason or "availability confirmed; no fault activated")
        except (ValueError, TypeError, AttributeError): return

    def blocked_activation(self, text):
        command, separator, value = text.partition("=")
        if separator and command in self.groups and not (self.entries[command]["enabled"] and self.entries[self.groups[command]]["enabled"]):
            try:
                if float(value) == 0: return ""
            except ValueError: pass
            return command
        return ""
