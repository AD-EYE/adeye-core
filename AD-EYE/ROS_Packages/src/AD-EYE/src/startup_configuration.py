"""Save settings for the next launch. Never change running ROS settings or send car commands."""
import hashlib
import json
import os
import stat
import tempfile
import threading
from contextlib import contextmanager

FORMAT = "adeye.test_configuration.startup"
SETTINGS = {
    "steering_proxy": "Steering proxy", "acceleration_proxy": "Acceleration proxy",
    "steering_faults": "Steering faults at startup", "acceleration_faults": "Acceleration faults at startup",
    "lidar_path": "LiDAR fault path", "gnss_path": "GNSS fault path",
    "recorded_steering": "Use replay steering", "recorded_acceleration": "Use replay acceleration",
    "goal_monitor": "Goal information monitor", "goal_stop": "Goal-stop action (live only)",
    "manual_recovery": "Allow manual software recovery", "recovery_monitor": "Recovery information monitor",
    "recording": "Record physical commands", "fault_timeout": "Actuator fault timeout",
}
DEFAULTS = {key: False for key in SETTINGS}
DEFAULTS.update(recovery_monitor=True, fault_timeout=True)
ARGUMENTS = dict(zip(
    ("steering_proxy", "acceleration_proxy", "steering_faults", "acceleration_faults", "lidar_path", "gnss_path",
     "recorded_steering", "recorded_acceleration", "goal_monitor", "goal_stop", "manual_recovery", "recovery_monitor", "recording", "fault_timeout"),
    ("enable_steering_proxy", "enable_acceleration_proxy", "enable_steering_faults", "enable_acceleration_faults", "enable_lidar_path", "enable_gnss_path",
     "use_recorded_steering", "use_recorded_acceleration", "enable_goal_stop_monitor", "enable_goal_stop", "enable_manual_recovery", "enable_vehicle_recovery_monitor", "record_physical_commands", "enable_physical_fault_timeout")))
MODULES = {
    "base": (), "proxy_only": ("steering_proxy", "acceleration_proxy"),
    "steering_faults": ("steering_proxy", "acceleration_proxy", "steering_faults"),
    "acceleration_faults": ("steering_proxy", "acceleration_proxy", "acceleration_faults"),
    "steering_acceleration_faults": ("steering_proxy", "acceleration_proxy", "steering_faults", "acceleration_faults"),
    "lidar_faults": ("lidar_path",), "gnss_faults": ("gnss_path",),
    "sensor_faults": ("lidar_path", "gnss_path"),
    "all_faults": ("steering_proxy", "acceleration_proxy", "steering_faults", "acceleration_faults", "lidar_path", "gnss_path"),
}


def default_path():
    return os.path.join(os.environ.get("ROS_HOME", os.path.expanduser("~/.ros")), "adeye", "test_configuration", "startup.json")


def validate(settings, complete=True):
    if not isinstance(settings, dict) or set(settings) != set(SETTINGS) or any(not isinstance(value, bool) for value in settings.values()):
        raise ValueError("Startup settings must contain exactly the supported boolean fields")
    errors = []
    for axis in ("steering", "acceleration"):
        if (settings[axis + "_faults"] or settings["recorded_" + axis]) and not settings[axis + "_proxy"]:
            errors.append("%s faults/replay require its proxy" % axis.capitalize())
    if settings["goal_stop"] and (settings["recorded_steering"] or settings["recorded_acceleration"]):
        errors.append("Goal-stop action is live-only; turn off replay or goal-stop")
    if complete and errors: raise ValueError("; ".join(errors))
    return errors


def read_file(path):
    if not os.path.lexists(path): return None, "absent"
    if stat.S_ISLNK(os.lstat(path).st_mode) or not os.path.isfile(path):
        raise ValueError("Startup override must be a regular file, not a link")
    with open(path, "rb") as stream: raw = stream.read(65537)
    if len(raw) > 65536: raise ValueError("Startup file is too large")
    value = json.loads(raw.decode("utf-8"))
    if not isinstance(value, dict) or set(value) != {"format", "schema_version", "settings"} or value.get("format") != FORMAT or value.get("schema_version") != 1:
        raise ValueError("Not a supported Test Configuration startup file; it will not be overwritten")
    validate(value["settings"])
    return value["settings"], hashlib.sha256(raw).hexdigest()


@contextmanager
def disk_lock(path):
    # Stop two manager sessions from saving over each other's settings at the same time.
    directory = os.path.dirname(path)
    if not os.path.isdir(directory):
        try: os.makedirs(directory, 0o700)
        except OSError:
            if not os.path.isdir(directory): raise
    lock_path = path + ".lock"
    if os.path.islink(lock_path): raise ValueError("Startup lock cannot be a link")
    with open(lock_path, "a+b") as stream:
        if os.name == "nt":
            import msvcrt
            if os.fstat(stream.fileno()).st_size == 0: stream.write(b"0"); stream.flush()
            stream.seek(0); msvcrt.locking(stream.fileno(), msvcrt.LK_NBLCK, 1)
        else:
            import fcntl
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        try: yield
        finally:
            if os.name == "nt":
                stream.seek(0); msvcrt.locking(stream.fileno(), msvcrt.LK_UNLCK, 1)
            else: fcntl.flock(stream.fileno(), fcntl.LOCK_UN)


class StartupStore(object):
    def __init__(self, path=None, launch_settings=None):
        self.path = os.path.abspath(path or default_path())
        self.lock = threading.RLock()
        self.revision, self.error, self.result = 0, "", "No startup change requested"
        self.saved, self.fingerprint = None, None
        self.launch_settings = dict(launch_settings) if launch_settings else None
        if self.launch_settings is not None: validate(self.launch_settings)
        try: self.saved, self.fingerprint = read_file(self.path)
        except (ValueError, OSError, IOError) as error: self.error = str(error)
        # Default values do not tell us which settings the running manager actually used.
        self.draft = dict(self.saved if self.saved is not None else self.launch_settings if self.launch_settings is not None else DEFAULTS)

    def check_disk(self):
        saved, fingerprint = read_file(self.path)
        if self.fingerprint is None or fingerprint != self.fingerprint:
            raise ValueError("Startup file changed externally or could not be read. Reopen the manager to review it; nothing overwritten")
        return saved

    def write(self):
        validate(self.draft)
        self.check_disk()
        with disk_lock(self.path): self._write_locked()

    def _write_locked(self):
        self.check_disk()
        directory = os.path.dirname(self.path)
        descriptor, temporary = tempfile.mkstemp(prefix=".startup-", suffix=".tmp", dir=directory)
        try:
            with os.fdopen(descriptor, "wb") as stream:
                stream.write((json.dumps(dict(format=FORMAT, schema_version=1, settings=self.draft), sort_keys=True, indent=2) + "\n").encode("utf-8"))
                stream.flush(); os.fsync(stream.fileno())
            self.check_disk()
            # Replace the file in one step so readers do not get a half-written file.
            # Use the matching Windows call when running local tests.
            getattr(os, "replace", os.rename)(temporary, self.path)
        finally:
            if os.path.exists(temporary): os.unlink(temporary)
        self.saved, self.fingerprint = read_file(self.path)

    def change(self, revision, operation, key=None, value=None):
        with self.lock:
            if revision != self.revision: raise ValueError("Startup draft changed; refresh status before retrying")
            if operation == "set":
                if key not in SETTINGS or not isinstance(value, bool): raise ValueError("Unsupported startup setting")
                self.draft[key] = value
            elif operation == "module":
                if key not in MODULES or value is not True: raise ValueError("Select a supported module")
                # Change which proxies, sensor paths and fault groups start.
                # Keep live/replay, recording and other choices unchanged.
                for field in ("steering_proxy", "acceleration_proxy", "steering_faults", "acceleration_faults", "lidar_path", "gnss_path"):
                    self.draft[field] = field in MODULES[key]
            elif operation == "discard":
                self.draft = dict(self.saved if self.saved is not None else self.launch_settings if self.launch_settings is not None else DEFAULTS)
            elif operation == "save": self.write()
            elif operation == "restore":
                self.check_disk()
                if self.saved is not None:
                    with disk_lock(self.path):
                        self.check_disk()
                        os.unlink(self.path)
                self.saved, self.fingerprint = None, "absent"
                self.draft = dict(DEFAULTS)
            else: raise ValueError("Unsupported startup operation")
            self.revision += 1
            self.result = {"save": "Saved for the next configured launch; running manager unchanged",
                "restore": "Owned saved override removed; next configured launch uses original defaults. Running manager unchanged",
                "discard": "Unsaved draft discarded; saved file and running manager unchanged"}.get(operation, "Draft changed only; not saved or applied")
            return self.result

    def status(self):
        with self.lock:
            disk_error = self.error
            try: self.check_disk()
            except (ValueError, OSError, IOError) as error: disk_error = str(error)
            return dict(revision=self.revision, path=self.path, settings=dict(SETTINGS), modules=sorted(MODULES),
                defaults=dict(DEFAULTS), draft=dict(self.draft), saved=None if self.saved is None else dict(self.saved),
                running_launch=None if self.launch_settings is None else dict(self.launch_settings),
                dirty=self.draft != (self.saved if self.saved is not None else self.launch_settings if self.launch_settings is not None else DEFAULTS),
                restart_required=self.saved is not None and self.saved != self.launch_settings or self.saved is None and self.launch_settings is not None and self.launch_settings != DEFAULTS,
                validation_errors=validate(self.draft, complete=False), disk_error=disk_error,
                result=self.result, manual_restart_only=True,
                launch_command="rosrun adeye launch_configured_manager.py",
                scope="Startup fields only; runtime individual-fault overrides are not saved")


class StartupServices(object):
    def __init__(self, rospy, owner, store):
        from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
        self.rospy, self.owner, self.store = rospy, owner, store
        self.SetBool, self.SetBoolResponse, self.Trigger, self.TriggerResponse = SetBool, SetBoolResponse, Trigger, TriggerResponse
        self.services, self.registered_revision = [], None
        self.last_request = None
        self.lock = threading.Lock()

    def call(self, revision, operation, key=None, value=None):
        response = self.SetBoolResponse if operation in ("set", "module") else self.TriggerResponse
        try: success, message = True, self.store.change(revision, operation, key, value)
        except Exception as error: success, message = False, str(error)
        self.last_request = dict(operation=operation, revision=revision, success=success, message=message)
        return response(success, message)

    def status(self):
        with self.lock:
            result = self.store.status()
            revision = result["revision"]
            root = "/adeye/test_configuration/session/%s/startup/r%s" % (self.owner, revision)
            if revision != self.registered_revision:
                old, created = self.services, []
                try:
                    for operation, keys in (("set", SETTINGS), ("module", MODULES)):
                        for key in sorted(keys):
                            created.append(self.rospy.Service(root + "/" + operation + "/" + key, self.SetBool,
                                lambda request, op=operation, key=key, rev=revision: self.call(rev, op, key, request.data)))
                    for operation in ("save", "discard", "restore"):
                        created.append(self.rospy.Service(root + "/" + operation, self.Trigger,
                            lambda request, op=operation, rev=revision: self.call(rev, op)))
                except Exception:
                    for service in created: service.shutdown("startup service registration incomplete")
                    raise
                self.services = created
                self.registered_revision = revision
                for service in old: service.shutdown("startup draft revision changed")
            result["service_root"] = root
            result["services_available"] = True
            result["last_request"] = self.last_request
            return result
