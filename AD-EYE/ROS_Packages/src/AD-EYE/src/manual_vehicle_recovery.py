#!/usr/bin/env python
"""Reset software only when requested. Does not clear faults stored by the car itself."""
import errno
import json
import os
import sys
import threading
import time
import uuid
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

SUPPORTED_REASONS = frozenset(["DRIVER_BRAKE_INTERVENTION", "DRIVER_ACCELERATOR_INTERVENTION", "STOP_COMMAND"])
CONTROL = "/adeye/vehicle_recovery_control"
TOKEN_PARAM = "/adeye/vehicle_recovery_token"
ACTORS = frozenset(["vicontroller", "controlswitch", "ctrlcmdrepublisher", "vehiclecontroller",
                   "steeringfaultproxylive", "steeringfaultproxyrecorded", "accelerationfaultproxylive",
                   "recordedaccelerationfaultproxy", "sensorfaultinjectionmanager", "lidarfaultinjectionmanager",
                   "gnssfaultinjectionmanager", "physicalgoalstopmonitor"])


def validate_reset(snapshot, confirmation):
    if confirmation is not True:
        raise ValueError("Confirm physical standstill, AD mode off and rosbag stopped before requesting reset")
    if not snapshot or snapshot.get("state") != "FAULT":
        raise ValueError("Reset is only available for a latched ros2can FAULT")
    if snapshot.get("reason") not in SUPPORTED_REASONS or snapshot.get("fault_entry_reason") not in SUPPORTED_REASONS:
        raise ValueError("Fault reason is not supported for manual software reset")
    if snapshot.get("manual_recovery_enabled") is not True or not snapshot.get("boot_id"):
        raise ValueError("ros2can manual recovery is not enabled or its code needs updating")
    validate_observations(snapshot)


def validate_observations(snapshot):
    """Check standstill and AD off, not whether stored intervention flags cleared."""
    from vehicle_recovery_monitor import finite_number
    speed = snapshot.get("speed_mps")
    if not finite_number(speed) or abs(speed) > 0.1:
        raise ValueError("A usable standstill speed reading is required")
    if snapshot.get("ad_mode_state") != 0:
        raise ValueError("Vehicle must report AD mode off (raw value 0)")
    # These flags can remain set after a driver releases the controls. They do
    # not approve driving, and must not prevent a requested software restart.
    # The car's intervention handling and emergency-stop code are unchanged.


def run_reset(backend):
    """Wait until old commands and pending tests are cleared before restarting communication."""
    try:
        backend.prepare()
        backend.restart()
        backend.initialize()
        backend.release()
    except Exception as error:
        backend.abort(str(error))
        return False, str(error)
    return True, "ros2can restarted in INITIALIZING. Intervention readings were information only; driver intervention handling is unchanged. Set a new live goal, or restart replay deliberately, before requesting AD mode."


class ManualVehicleRecovery(object):
    def __init__(self):
        import rospy
        from std_msgs.msg import String
        from std_srvs.srv import SetBool, SetBoolResponse
        self.rospy, self.String, self.Response = rospy, String, SetBoolResponse
        self.lock = threading.RLock()
        self.operation_lock = threading.Lock()
        self.snapshot = None
        self.acks, self.pending_gui = {}, set()
        self.token, self.phase, self.detail = "", "idle", "Manual software reset is available on explicit request only; stored intervention flags do not prevent it"
        self.required = set()
        self.old_pid = None
        self.old_boot = None
        # Set up roslaunch on the main thread so it can handle shutdown correctly.
        import roslaunch
        self.launcher = roslaunch.scriptapi.ROSLaunch()
        self.launcher.start()
        self.failed_ack = False
        self.control = rospy.Publisher(CONTROL, String, queue_size=10, latch=True)
        self.status_pub = rospy.Publisher("/adeye/vehicle_recovery_operation_status", String, queue_size=1, latch=True)
        self.subs = [
            rospy.Subscriber("/adeye/vehicle_recovery_inputs", String, self.inputs, queue_size=1),
            rospy.Subscriber("/adeye/vehicle_recovery_ack", String, self.ack, queue_size=50),
            rospy.Subscriber("/adeye/vehicle_recovery_gui_state", String, self.gui_state, queue_size=50)]
        self.service = rospy.Service("/adeye/recover_to_ready", SetBool, self.request)
        self.timer = rospy.Timer(rospy.Duration(1.0), lambda event: self.publish_status())
        rospy.on_shutdown(self.close)
        self.publish_status()

    def inputs(self, message):
        try:
            from vehicle_recovery_monitor import read_snapshot
            snapshot = read_snapshot(message.data)
            with self.lock:
                self.snapshot = snapshot
        except (ValueError, TypeError, KeyError):
            with self.lock:
                self.snapshot = None

    def gui_state(self, message):
        try:
            state = json.loads(message.data)
            component = state["component"]
            if not isinstance(component, (str, type(u""))) or not component.startswith("gui-"):
                return
            with self.lock:
                if state.get("pending") is True:
                    self.pending_gui.add(component)
                else:
                    self.pending_gui.discard(component)
        except (ValueError, TypeError, KeyError):
            pass

    def ack(self, message):
        try:
            value = json.loads(message.data)
            with self.lock:
                if value.get("token") == self.token:
                    self.acks[(value["component"], value["phase"])] = value
                    if value["phase"] == "failed":
                        self.failed_ack = True
        except (ValueError, TypeError, KeyError):
            pass

    def publish_status(self):
        with self.lock:
            status = {"phase": self.phase, "detail": self.detail, "token": self.token,
                      "held": bool(self.token and self.phase != "complete"),
                      "enabled": True, "automatic_recovery": False,
                      "requires_operator_confirmation": True,
                      "intervention_flags_informational_only": True,
                      "intervention_flags": {name: (self.snapshot or {}).get(name) for name in
                                             ["brake_interrupt", "accelerator_interrupt", "steering_interrupt"]},
                      "waiting_for_cleanup": sorted(name for name in self.required if (name, "prepared") not in self.acks) if self.phase == "preparing" else [],
                      "can_sample_freshness_verified": False,
                      "wall_time_s": time.time()}
        self.status_pub.publish(self.String(data=json.dumps(status)))

    def stage(self, phase, detail):
        with self.lock:
            self.phase, self.detail = phase, detail
        self.publish_status()

    def send(self, action):
        self.control.publish(self.String(data=action + ":" + self.token))

    def wait(self, predicate, action=None, seconds=15.0):
        clock = getattr(time, "monotonic", time.time)
        end = clock() + seconds
        next_send = 0
        event = threading.Event()
        while not self.rospy.is_shutdown() and clock() < end:
            with self.lock:
                if self.failed_ack:
                    raise RuntimeError("A component refused recovery or emergency stop was requested")
                if predicate():
                    return
            if action and clock() >= next_send:
                self.send(action)
                next_send = clock() + 0.25
            event.wait(0.05)
        missing = sorted(name for name in self.required if (name, "prepared") not in self.acks) if self.phase == "preparing" else []
        raise RuntimeError("Manual recovery step was not confirmed: " + self.phase + ("; missing: " + ", ".join(missing) if missing else ""))

    def request(self, request):
        if not self.operation_lock.acquire(False):
            return self.Response(success=False, message="A manual reset is already in progress")
        try:
            with self.lock:
                validate_reset(self.snapshot, request.data)
                self.old_boot = self.snapshot["boot_id"]
            # Restart only ros2can on this computer, never a process on another computer.
            import rosnode
            import rosgraph
            import socket
            try:
                from xmlrpc.client import ServerProxy
            except ImportError:
                from xmlrpclib import ServerProxy
            try:
                from urllib.parse import urlparse
            except ImportError:
                from urlparse import urlparse
            master = rosgraph.Master(self.rospy.get_name())
            uri = rosnode.get_api_uri(master, "/vi_controller")
            if not uri:
                raise RuntimeError("ros2can node /vi_controller is not available")
            local_addresses = set()
            for host in [socket.gethostname(), "localhost", os.environ.get("ROS_IP"), os.environ.get("ROS_HOSTNAME")]:
                if host:
                    try:
                        local_addresses.update(item[4][0] for item in socket.getaddrinfo(host, None))
                    except socket.gaierror:
                        pass
            remote_addresses = {item[4][0] for item in socket.getaddrinfo(urlparse(uri).hostname, None)}
            if not remote_addresses.intersection(local_addresses):
                raise RuntimeError("Recovery cannot restart ros2can on a different computer")
            code, _, pid = ServerProxy(uri).getPid(self.rospy.get_name())
            if code != 1 or not isinstance(pid, int) or pid <= 1 or pid == os.getpid():
                raise RuntimeError("Could not verify the ros2can process")
            if not self.old_boot.startswith(str(pid) + "-"):
                raise RuntimeError("Recovery inputs do not belong to the running ros2can process")
            with open("/proc/{0}/cmdline".format(pid), "rb") as process:
                arguments = [part.decode("utf-8") for part in process.read().split(b"\0") if part]
                executable = arguments[0]
            if os.path.basename(executable) != "vi_controller":
                raise RuntimeError("Recovery only supports a local vi_controller process")
            self.old_pid = pid
            try:
                from shlex import quote
            except ImportError:
                from pipes import quote
            self.restart_args = " ".join(quote(arg) for arg in arguments[1:] if not arg.startswith(("__name:=", "__log:=")))
            nodes = rosnode.get_node_names()
            self.required = {node for node in nodes if node.rsplit("/", 1)[-1].replace("_", "").lower() in ACTORS}
            # Also check command-sending nodes with unfamiliar names.
            # Keep the reset on hold if a node cannot confirm that its old commands were cleared.
            guarded_topics = {"/steering_requested_phy", "/acceleration_requested_phy",
                              "/nominal/steering_requested_phy", "/nominal/acceleration_requested_phy",
                              "/TwistS", "/old_steering_requested_phy",
                              "/adeye/goal_stop_speed_limit", "/adeye/goal_stop_acceleration"}
            publishers, _, _ = master.getSystemState()
            for topic, producers in publishers:
                if topic in guarded_topics:
                    self.required.update(producers)
            self.required.add("/vi_controller")
            with self.lock:
                self.required.update(self.pending_gui)
                self.acks = {}
                self.failed_ack = False
                self.token = uuid.uuid4().hex
            self.rospy.set_param(TOKEN_PARAM, self.token)
            success, detail = run_reset(self)
            if success:
                self.stage("complete", detail)
            return self.Response(success=success, message=detail)
        except Exception as error:
            return self.Response(success=False, message=str(error))
        finally:
            self.operation_lock.release()

    def prepare(self):
        self.stage("preparing", "Holding command sources and cancelling tests; waiting for cleanup acknowledgements")
        self.wait(lambda: all((component, "prepared") in self.acks for component in self.required), "PREPARE")
        validate_observations(self.snapshot or {})

    def restart(self):
        import rosnode
        import roslaunch
        self.stage("restarting", "Restarting only ros2can to recreate CAN receivers and software state")
        success, failed = rosnode.kill_nodes(["/vi_controller"])
        if failed or "/vi_controller" not in success:
            raise RuntimeError("ros2can did not accept shutdown")
        def stopped():
            try:
                os.kill(self.old_pid, 0)
                return False
            except OSError as error:
                if error.errno == errno.ESRCH:
                    return True
                raise
        self.wait(stopped)
        # Keep ros2can's saved ROS settings. Do not automatically restart it again if it exits.
        self.process = self.launcher.launch(roslaunch.core.Node("can_vehicle", "vi_controller", name="vi_controller", output="screen", args=self.restart_args))
        def booted():
            ack = self.acks.get(("/vi_controller", "booted"), {})
            return bool(ack.get("boot_id") and ack["boot_id"] != self.old_boot)
        self.wait(booted, "PREPARE")

    def initialize(self):
        self.stage("initializing", "Restoring startup CAN communication while commands remain held; stored intervention flags are information only")
        self.wait(lambda: ("/vi_controller", "initialized") in self.acks, "INITIALIZE")
        def ready_observation():
            if not self.snapshot or self.snapshot.get("boot_id") == self.old_boot:
                return False
            if self.snapshot.get("state") != "INITIALIZING":
                return False
            try:
                validate_observations(self.snapshot)
                return True
            except ValueError:
                return False
        self.wait(ready_observation)

    def release(self):
        self.stage("releasing", "Finishing in INITIALIZING; live command sources require a new RViz goal")
        # Panels confirm that pending tests were cancelled; they do not release the reset hold.
        components = {name for name in self.required if not name.startswith("gui-")}
        self.wait(lambda: all((name, "released") in self.acks for name in components), "RELEASE")
        self.rospy.set_param(TOKEN_PARAM, "")

    def abort(self, detail):
        self.send("PREPARE")
        self.send("ABORT")
        self.stage("failed", "Reset failed; command hold remains active. " + detail)

    def close(self):
        if self.launcher is not None:
            self.launcher.stop()


if __name__ == "__main__":
    import rospy
    rospy.init_node("manual_vehicle_recovery")
    ManualVehicleRecovery()
    rospy.spin()
