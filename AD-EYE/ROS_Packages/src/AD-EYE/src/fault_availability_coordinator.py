"""Manage fault switches for this session. Never change steering or acceleration values."""
import json
import threading
import time
import uuid

STATE_TOPIC = "/adeye/fault_availability_state"
CONTROL_TOPIC = "/adeye/fault_availability_control"
SERVICE_ROOT = "/adeye/test_configuration"
GROUPS = {"steering_fault_processing": "STEER_GROUP_command",
          "acceleration_fault_processing": "ACCEL_GROUP_command",
          "lidar_fault_processing": "LIDAR_GROUP_command",
          "gnss_fault_processing": "GNSS_GROUP_command"}


def parent_group(command):
    return next((group for group in GROUPS.values() if command.startswith(group.split("_")[0])), None)
try:
    string_types = (basestring,)
except NameError:
    string_types = (str,)


def read_state(text, known):
    data = json.loads(text)
    if not isinstance(data, dict) or data.get("schema_version") != 1:
        raise ValueError("invalid availability envelope")
    for field in ("component", "boot_id", "detail", "request_id"):
        if not isinstance(data.get(field), string_types): raise ValueError("invalid identity")
    if not data["component"].startswith("/") or not data["boot_id"] or not isinstance(data.get("busy"), bool) or not isinstance(data.get("accepted"), bool):
        raise ValueError("invalid availability state")
    commands = data.get("commands")
    if not isinstance(commands, dict) or not commands or not set(commands).issubset(known):
        raise ValueError("unsupported command inventory")
    for entry in commands.values():
        if not isinstance(entry, dict) or not all(isinstance(entry.get(key), bool) for key in ("baseline", "enabled")):
            raise ValueError("invalid switch")
        if isinstance(entry.get("revision"), bool) or not isinstance(entry.get("revision"), int) or entry["revision"] < 0 or not isinstance(entry.get("owner"), string_types):
            raise ValueError("invalid revision/owner")
        if not entry["owner"] and entry["enabled"] != entry["baseline"]: raise ValueError("unowned override")
    return data


class AvailabilityCoordinator(object):
    def __init__(self, rospy, String, registry, owner):
        from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
        self.rospy, self.String = rospy, String
        self.SetBoolResponse, self.TriggerResponse, self.Trigger = SetBoolResponse, TriggerResponse, Trigger
        self.owner = owner
        self.service_root = SERVICE_ROOT + "/session/" + owner
        self.commands = {item["command"] for item in registry["features"] if item["kind"] == "fault_availability"}
        self.known = self.commands | set(GROUPS.values())
        self.condition = threading.Condition(threading.RLock())
        self.operation_lock = threading.Lock()
        self.states, self.acks, self.reservations, self.release_services, self.pending_gui = {}, {}, {}, {}, {}
        self.retired_boots = set()
        self.started = set()
        self.completed = []
        self.clock = getattr(time, "monotonic", time.time)
        self.pub = rospy.Publisher(CONTROL_TOPIC, String, queue_size=20)  # Do not save and resend this request to new subscribers.
        self.sub = rospy.Subscriber(STATE_TOPIC, String, self.state_callback, queue_size=50)
        self.pending_sub = rospy.Subscriber("/adeye/vehicle_recovery_gui_state", String, self.pending_callback, queue_size=50)
        self.services = []
        for command in sorted(self.commands):
            self.services.append(rospy.Service(self.service_root + "/fault/" + command, SetBool,
                lambda request, command=command: self.set_service(command, request.data)))
            self.services.append(rospy.Service(self.service_root + "/reserve/" + command, Trigger,
                lambda request, command=command: self.reserve_service(command)))
        for feature, command in sorted(GROUPS.items()):
            self.services.append(rospy.Service(self.service_root + "/group/" + feature, SetBool,
                lambda request, command=command: self.set_service(command, request.data)))
        self.services.append(rospy.Service(self.service_root + "/reset_fault_overrides", Trigger, self.reset_service))

    def state_callback(self, message):
        try: state = read_state(message.data, self.known)
        except (ValueError, TypeError, AttributeError): return
        with self.condition:
            if (state["component"], state["boot_id"]) in self.retired_boots: return
            previous = self.states.get(state["component"])
            if previous and previous[1]["boot_id"] != state["boot_id"]:
                self.retired_boots.add((state["component"], previous[1]["boot_id"]))
            if previous and previous[1]["boot_id"] == state["boot_id"]:
                if any(entry["revision"] < previous[1]["commands"].get(command, entry)["revision"] for command, entry in state["commands"].items()): return
            self.states[state["component"]] = (self.clock(), state)
            if state["request_id"] in self.acks:
                self.acks[state["request_id"]] = state
            getattr(self.condition, "notify_all", self.condition.notifyAll)()

    def pending_callback(self, message):
        try:
            value = json.loads(message.data)
            component, pending = value.get("component"), value.get("pending")
            if not isinstance(component, string_types) or not isinstance(pending, bool): return
            with self.condition:
                if pending: self.pending_gui[component] = value.get("fault") if value.get("fault") in self.commands else None
                else: self.pending_gui.pop(component, None)
        except (ValueError, TypeError, AttributeError): return

    def resolve(self, command):
        found = [state for _, state in self.states.values() if command in state["commands"]]
        if len(found) != 1: raise ValueError("No unique confirmed controller for this fault; check module and publishers")
        return found[0]

    def check_pending(self, command):
        affected = {fault for fault in self.commands if parent_group(fault) == command} if command in GROUPS.values() else {command}
        if affected.intersection(self.reservations) or any(fault is None or fault in affected for fault in self.pending_gui.values()):
            raise ValueError("Cancel/finish the affected pending or active random test first")

    def effective_enabled(self, command, state):
        parent = parent_group(command)
        # Older nodes have no group switch; keep using their individual fault switches.
        return state["commands"][command]["enabled"] and state["commands"].get(parent, {"enabled": True})["enabled"]

    def change(self, command, operation, value=None):
        # operation_lock keeps a random test from being scheduled during this change.
        with self.condition:
            self.check_pending(command)
            state = self.resolve(command)
            entry = state["commands"][command]
            if state["busy"]: raise ValueError("Controller is busy; stop its test/recovery first")
            if entry["owner"] not in ("", self.owner): raise ValueError("Override belongs to another session; do not overwrite it")
            if operation == "reset" and entry["owner"] != self.owner:
                return "No owned override; nothing changed"
            if operation == "set" and entry["enabled"] == value:
                return "Already confirmed; no change requested"
            request_id = uuid.uuid4().hex
            request = dict(schema_version=1, component=state["component"], boot_id=state["boot_id"],
                revision=entry["revision"], owner=self.owner, request_id=request_id,
                fault=command, operation=operation, value=value)
            self.acks[request_id] = None
            self.pub.publish(self.String(data=json.dumps(request)))
            deadline = self.clock() + 3.0
            while self.acks[request_id] is None and self.clock() < deadline:
                self.condition.wait(min(0.1, max(0, deadline - self.clock())))
            acknowledgement = self.acks.pop(request_id)
            if acknowledgement is None:
                raise ValueError("No matching acknowledgement; outcome unknown. Inspect confirmed state before retrying")
            if not acknowledgement["accepted"]: raise ValueError(acknowledgement["detail"])
            return acknowledgement["detail"]

    def set_service(self, command, value):
        if not self.operation_lock.acquire(False): return self.SetBoolResponse(False, "Another configuration request is in progress")
        try:
            if not isinstance(value, bool): raise ValueError("boolean required")
            return self.SetBoolResponse(True, self.change(command, "set", value))
        except Exception as error: return self.SetBoolResponse(False, str(error))
        finally: self.operation_lock.release()

    def reset_service(self, _request):
        if not self.operation_lock.acquire(False): return self.TriggerResponse(False, "Another configuration request is in progress")
        try:
            with self.condition:
                owned = sorted(command for _, state in self.states.values() for command, entry in state["commands"].items() if entry["owner"] == self.owner)
                # Restore groups that started Off first, and groups that started On last.
                owned.sort(key=lambda command: (0 if command in GROUPS.values() and not self.resolve(command)["commands"][command]["baseline"] else 2 if command in GROUPS.values() else 1, command))
                # Check every setting changed by this session before starting the reset.
                for command in owned:
                    self.check_pending(command)
                    if self.resolve(command)["busy"]: raise ValueError("Controller busy; stop its test/recovery first")
            done = []
            for command in owned:
                try: self.change(command, "reset"); done.append(command)
                except Exception as error:
                    return self.TriggerResponse(False, "Reset incomplete. Confirmed resets: %s. %s" % (", ".join(done) or "none", error))
            return self.TriggerResponse(True, "Owned fault-availability overrides removed" if owned else "No owned overrides; nothing changed")
        except Exception as error: return self.TriggerResponse(False, str(error))
        finally: self.operation_lock.release()

    def reserve_service(self, command):
        if not self.operation_lock.acquire(False): return self.TriggerResponse(False, "Configuration change in progress; try arming again")
        try:
            with self.condition:
                self.check_pending(command)
                state = self.resolve(command)
                if state["busy"]: raise ValueError("Controller busy; stop its test/recovery before scheduling")
                if not self.effective_enabled(command, state): raise ValueError("Fault or its processing group is disabled by Test Configuration")
                token = uuid.uuid4().hex
                self.reservations[command] = token
                endpoint = SERVICE_ROOT + "/release/" + token
                self.release_services[token] = self.rospy.Service(endpoint, self.Trigger,
                    lambda request: self.release(command, token))
                return self.TriggerResponse(True, endpoint)
        except Exception as error: return self.TriggerResponse(False, str(error))
        finally: self.operation_lock.release()

    def release(self, command, token):
        with self.operation_lock:
            with self.condition:
                if token in self.completed: return self.TriggerResponse(True, "Reservation already released")
                if self.reservations.get(command) != token: return self.TriggerResponse(False, "Reservation does not match")
                del self.reservations[command]
                self.started.discard(command)
                # Remember a few finished tests so a second release request is harmless.
                # The panel and ROS node may both report that the same test has ended.
                self.completed.append(token)
                service = self.release_services.pop(self.completed.pop(0), None) if len(self.completed) > 64 else None
            if service: service.shutdown("test cancelled or finished")
        return self.TriggerResponse(True, "Reservation released; no fault command sent")

    def fault_report(self, report):
        # Once a test ends, remove its scheduling block even if its panel was closed.
        command, event = report.get("fault"), report.get("event")
        token = None
        with self.condition:
            if command not in self.reservations: return
            if report.get("active") is True: self.started.add(command)
            elif command in self.started and event in ("reset", "timeout"):
                token = self.reservations[command]
        if token: self.release(command, token)

    def status(self):
        with self.condition:
            now, commands, groups, unconfirmed = self.clock(), {}, {}, {}
            for command in sorted(self.known):
                try:
                    state = self.resolve(command)
                    entry = dict(state["commands"][command])
                    received = self.states[state["component"]][0]
                    confirmed = dict(entry, component=state["component"], boot_id=state["boot_id"], busy=state["busy"],
                        received_age_s=max(0, now - received), reserved=command in self.reservations,
                        owned_by_this_session=entry["owner"] == self.owner)
                    if command in GROUPS.values():
                        confirmed["reserved"] = any(parent_group(fault) == command for fault in self.reservations)
                        groups[next(feature for feature, group in GROUPS.items() if group == command)] = confirmed
                    else:
                        confirmed["effective_enabled"] = self.effective_enabled(command, state)
                        commands[command] = confirmed
                except ValueError:
                    count = sum(command in state["commands"] for _, state in self.states.values())
                    unconfirmed[command] = "No controller acknowledgement received; module may be absent" if count == 0 else "Multiple controllers claim this switch; configuration is ambiguous"
            return dict(enabled=True, session_id=self.owner, commands=commands, groups=groups, unconfirmed=unconfirmed,
                        owned_override_count=sum(entry["owner"] == self.owner for _, state in self.states.values() for entry in state["commands"].values()),
                        reservation_count=len(self.reservations), pending_panel_count=len(self.pending_gui),
                        scope="individual and group fault availability only; no other settings are reset", persistence="session_only")
