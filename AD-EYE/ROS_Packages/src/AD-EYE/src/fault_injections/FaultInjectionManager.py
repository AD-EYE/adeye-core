#!/usr/bin/env python

import datetime
import glob
import gzip
import math
import os
import random
import shutil
import threading
import time

import rospy
from std_msgs.msg import String
from autoware_msgs.msg import VehicleCmd


class FaultInjectionManager:

    FAULT_COMMAND_PREFIXES = (
        "HL_command=",
        "WLOCK_command=",
        "STEEROFFSET_command=",
        "STEERFREEZE_command=",
        "STEERSAT_command=",
        "STEEROSC_command=",
        "STEERRANDOM_command=",
        "ACCELOFFSET_command=",
        "ACCELFREEZE_command=",
        "ACCELSAT_command=",
        "ACCELOSC_command=",
        "ACCELRUNAWAY_command=",
        "GNSS_",
        "LIDAR_",
    )

    def __init__(self, manager_state_machine):

        log_dir = rospy.get_param(
            "~fault_log_dir",
            os.path.join(
                os.path.expanduser("~"),
                ".ros",
                "adeye",
                "fault_logs",
            ),
        )

        if not os.path.isdir(log_dir):
            try:
                os.makedirs(log_dir)
            except OSError:
                if not os.path.isdir(log_dir):
                    raise

        self.fault_log_dir = log_dir
        self.fault_log_max_bytes = max(
            0, int(rospy.get_param("~fault_log_max_bytes", 10 * 1024 * 1024))
        )
        self.fault_log_keep_files = max(
            1, int(rospy.get_param("~fault_log_keep_files", 20))
        )
        self.fault_log_heartbeat_s = max(
            0.0, float(rospy.get_param("~fault_log_heartbeat_s", 5.0))
        )
        self.fault_log_value_change_min_interval_s = max(
            0.0,
            float(
                rospy.get_param(
                    "~fault_log_value_change_min_interval_s", 1.0
                )
            ),
        )
        self.fault_log_compress_rotated = rospy.get_param(
            "~fault_log_compress_rotated", True
        )
        self._last_fault_log_entries = {}
        self._open_new_fault_log()
        self._prune_fault_logs()
        rospy.on_shutdown(self._close_fault_log)
        self.VehicleStates = self.VehicleStates()
        self._republishing_lock = threading.Lock()

        self.fault_debug_pub = rospy.Publisher(
            "/adeye/fault_log", String, queue_size=50
        )

        self.manager_state_machine = manager_state_machine
        self.republishing_vehicle_cmd = False

        self.send_wheel_state_pub = rospy.Publisher(
            "/vehicle_cmd", VehicleCmd, queue_size=1
        )

        self.send_vehicle_commands_pub = rospy.Publisher(
            "/vehicle_commands", String, queue_size=1
        )

    class VehicleStates:
        def __init__(self):
            self.lamps = 0
            self.speed_gui = 0.0
            self.steer_gui = 0.0
            self.steer_being_kept = None
            self.hl_gui = 0
            self.wheel_gui = 1
            self.accelerate_gui = 0.0

            # Steering faults
            self.steer_offset_gui = 0.0
            self.steer_freeze_gui = 0
            self.steer_frozen_value = None
            self.steer_saturation_gui = 0.0
            self.steer_oscillation_gui = 0.0
            self.steer_random_gui = 0

            # Accelerating faults
            self.accelerate_offset_gui = 0.0
            self.accelerate_freeze_gui = 0
            self.accelerate_frozen_value = None
            self.accelerate_saturation_gui = 0.0
            self.accelerate_oscillation_gui = 0.0
            self.accelerate_runaway_gui = 0

            self.current_speed = 0.0
            self.current_angle = 0.0
            self.current_lamp = 0

    @staticmethod
    def _parse_command_value(command, prefix):
        if not command.startswith(prefix):
            return None

        try:
            return float(command[len(prefix) :])
        except ValueError:
            rospy.logwarn("Invalid value for command: %s", command)
            return None

    def logFaultDebug(self, fault, value):

        # rostopic echo /adeye/fault_log

        msg = fault + ": " + str(value)

        self.fault_debug_pub.publish(msg)

        rospy.loginfo(msg)

    def _new_fault_log_path(self):
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        filename = "adeye_fault_log_{}_{}.txt".format(timestamp, os.getpid())
        return os.path.join(self.fault_log_dir, filename)

    def _open_new_fault_log(self):
        self.fault_log_path = self._new_fault_log_path()
        self.fault_log = open(self.fault_log_path, "a")

    def _compress_fault_log(self, path):
        if not self.fault_log_compress_rotated or not os.path.isfile(path):
            return

        compressed_path = path + ".gz"
        try:
            with open(path, "rb") as source, gzip.open(
                compressed_path, "wb"
            ) as target:
                shutil.copyfileobj(source, target)
            os.remove(path)
        except (IOError, OSError) as error:
            rospy.logwarn("Could not compress fault log %s: %s", path, error)

    def _prune_fault_logs(self):
        paths = glob.glob(
            os.path.join(self.fault_log_dir, "adeye_fault_log_*.txt*")
        )
        paths.sort(key=os.path.getmtime, reverse=True)

        for path in paths[self.fault_log_keep_files :]:
            if path != self.fault_log_path:
                try:
                    os.remove(path)
                except OSError as error:
                    rospy.logwarn("Could not remove old fault log %s: %s", path, error)

    def _rotate_fault_log_if_needed(self):
        if self.fault_log_max_bytes <= 0:
            return

        self.fault_log.flush()
        if os.path.getsize(self.fault_log_path) < self.fault_log_max_bytes:
            return

        previous_path = self.fault_log_path
        self.fault_log.close()
        self._compress_fault_log(previous_path)
        self._open_new_fault_log()
        self._prune_fault_logs()

    def _close_fault_log(self):
        if not self.fault_log.closed:
            self.fault_log.close()

    def _should_log_fault(self, fault, value, force):
        now = time.time()
        value = str(value)
        previous = self._last_fault_log_entries.get(fault)

        if force or previous is None:
            self._last_fault_log_entries[fault] = (value, now)
            return True

        previous_value, previous_time = previous
        value_changed = value != previous_value
        elapsed = now - previous_time
        if value_changed and elapsed >= self.fault_log_value_change_min_interval_s:
            self._last_fault_log_entries[fault] = (value, now)
            return True

        if elapsed >= self.fault_log_heartbeat_s:
            self._last_fault_log_entries[fault] = (value, now)
            return True

        return False

    def logFault(self, fault, value, force=False):
        """Log fault transitions plus bounded periodic heartbeat records."""

        if not self._should_log_fault(fault, value, force):
            return

        timestamp = datetime.datetime.now()

        line = str(timestamp) + " | " + fault + " | " + str(value) + "\n"

        self.fault_log.write(line)
        self.fault_log.flush()
        self._rotate_fault_log_if_needed()

        rospy.loginfo(line)

    ##########################################################################
    # EMERGENCY STATE CALLBACK
    ##########################################################################

    def emergencyCallback(self, msg):

        if msg.data == "emergency":
            rospy.loginfo("Entering Fault state")
            self.manager_state_machine.current_state = (
                self.manager_state_machine.States.FAULT_STATE
            )

        elif msg.data == "return_to_ready":
            rospy.loginfo("Entering Enabled state from Fault state")
            self.manager_state_machine.current_state = (
                self.manager_state_machine.States.ENABLED_STATE
            )

        elif msg.data == "return_from_emergency":
            rospy.loginfo("Entering Initializing state from Fault state")
            self.manager_state_machine.current_state = (
                self.manager_state_machine.States.INITIALIZING_STATE
            )

    ##########################################################################
    # VEHICLE STATUS CALLBACK
    ##########################################################################

    def vehicleStatusCallback(self, msg):
        """Receive simulation status from autoware_msgs/VehicleStatus."""

        self.vehicleSpeedCallback(msg.speed)
        self.vehicleSteeringAngleCallback(msg.angle)

    def vehicleSpeedCallback(self, speed):
        """Receive the current longitudinal speed in metres per second."""

        self.VehicleStates.current_speed = speed
        #######################################################################
        # Wheel lock
        #######################################################################

        if self.VehicleStates.wheel_gui == 0:

            if abs(speed) < 0.1:

                if self.VehicleStates.steer_being_kept is None:

                    self.VehicleStates.steer_being_kept = self.VehicleStates.steer_gui

                    rospy.loginfo(
                        "Wheel lock engaged at steering angle %.2f",
                        self.VehicleStates.steer_being_kept,
                    )

            else:

                rospy.logwarn("Wheel lock requested while vehicle is moving.")

        else:

            self.VehicleStates.steer_being_kept = None

    def vehicleSteeringAngleCallback(self, angle):
        """Receive the current steering angle from the vehicle interface."""

        self.VehicleStates.current_angle = angle

    ##########################################################################
    # VEHICLE COMMAND CALLBACK
    ##########################################################################

    def vehicleCommandCallback(self, msg):

        if msg.data.startswith(self.FAULT_COMMAND_PREFIXES):
            self.logFault("FAULT_COMMAND", msg.data, force=True)

        if msg.data == "HL_command=1":

            if self.VehicleStates.hl_gui != 1:
                self.VehicleStates.hl_gui = 1
                rospy.loginfo("Hazard lights on from GUI")

        elif msg.data == "HL_command=0":

            if self.VehicleStates.hl_gui != 0:
                self.VehicleStates.hl_gui = 0
                rospy.loginfo("Hazard lights off from GUI")

        elif msg.data == "WLOCK_command=0":

            if self.VehicleStates.wheel_gui != 0:
                self.VehicleStates.wheel_gui = 0
                rospy.loginfo("Wheel lock on from GUI")

        elif msg.data == "WLOCK_command=1":

            if self.VehicleStates.wheel_gui != 1:
                self.VehicleStates.wheel_gui = 1
                rospy.loginfo("Wheel lock off from GUI")

        b = self._parse_command_value(msg.data, "ACCELERATE_command=")
        if b is not None:

            if self.VehicleStates.speed_gui != b:

                self.VehicleStates.speed_gui = b

                rospy.loginfo("Setting accelerate from GUI")
                rospy.loginfo(self.VehicleStates.speed_gui)

        b = self._parse_command_value(msg.data, "STEERING_command=")
        if b is not None:

            if self.VehicleStates.steer_gui != b:

                self.VehicleStates.steer_gui = b

                rospy.loginfo("Setting steer from GUI")
                rospy.loginfo(self.VehicleStates.steer_gui)

        b = self._parse_command_value(msg.data, "STEEROFFSET_command=")
        if b is not None:

            if self.VehicleStates.steer_offset_gui != b:

                self.VehicleStates.steer_offset_gui = b

                rospy.loginfo("Setting steering offset from GUI")

                rospy.loginfo(self.VehicleStates.steer_offset_gui)
        if msg.data == "STEERFREEZE_command=1":
            self.VehicleStates.steer_freeze_gui = 1

        if msg.data == "STEERFREEZE_command=0":
            self.VehicleStates.steer_freeze_gui = 0
        if msg.data == "STEERRANDOM_command=1":
            self.VehicleStates.steer_random_gui = 1

        if msg.data == "STEERRANDOM_command=0":
            self.VehicleStates.steer_random_gui = 0

        b = self._parse_command_value(msg.data, "STEERSAT_command=")
        if b is not None:

            if self.VehicleStates.steer_saturation_gui != b:

                self.VehicleStates.steer_saturation_gui = b

                rospy.loginfo("Setting steering saturation from GUI")

                rospy.loginfo(self.VehicleStates.steer_saturation_gui)

        b = self._parse_command_value(msg.data, "STEEROSC_command=")
        if b is not None:

            if self.VehicleStates.steer_oscillation_gui != b:

                self.VehicleStates.steer_oscillation_gui = b

                rospy.loginfo("Setting steering oscillation from GUI")

                rospy.loginfo(self.VehicleStates.steer_oscillation_gui)

        # Acceleration offset

        b = self._parse_command_value(msg.data, "ACCELOFFSET_command=")
        if b is not None:

            if self.VehicleStates.accelerate_offset_gui != b:

                self.VehicleStates.accelerate_offset_gui = b

                rospy.loginfo("Setting acceleration offset from GUI")

                rospy.loginfo(self.VehicleStates.accelerate_offset_gui)

        # Acceleration freeze

        if msg.data == "ACCELFREEZE_command=1":

            self.VehicleStates.accelerate_freeze_gui = 1

            rospy.loginfo("Acceleration freeze ON")

        elif msg.data == "ACCELFREEZE_command=0":

            self.VehicleStates.accelerate_freeze_gui = 0

            self.VehicleStates.accelerate_frozen_value = None

            rospy.loginfo("Acceleration freeze OFF")

        # Acceleration saturation

        b = self._parse_command_value(msg.data, "ACCELSAT_command=")
        if b is not None:

            if self.VehicleStates.accelerate_saturation_gui != b:

                self.VehicleStates.accelerate_saturation_gui = b

                rospy.loginfo("Setting acceleration saturation")

                rospy.loginfo(self.VehicleStates.accelerate_saturation_gui)

        # Acceleration oscillation

        b = self._parse_command_value(msg.data, "ACCELOSC_command=")
        if b is not None:

            if self.VehicleStates.accelerate_oscillation_gui != b:

                self.VehicleStates.accelerate_oscillation_gui = b

                rospy.loginfo("Setting acceleration oscillation")

                rospy.loginfo(self.VehicleStates.accelerate_oscillation_gui)
        # Runaway acceleration

        if msg.data == "ACCELRUNAWAY_command=1":

            self.VehicleStates.accelerate_runaway_gui = 1

            rospy.logwarn("Runaway acceleration ON")

        elif msg.data == "ACCELRUNAWAY_command=0":

            self.VehicleStates.accelerate_runaway_gui = 0

            rospy.logwarn("Runaway acceleration OFF")

    ##########################################################################
    # VEHICLE CMD FAULT CALLBACK
    ##########################################################################

    def vehicleCmdFaultInjectionCallback(self, msg):

        if self.republishing_vehicle_cmd:
            return

        modified = False
        fault_active = (
            self.VehicleStates.hl_gui == 1
            or self.VehicleStates.wheel_gui == 0
            or self.VehicleStates.steer_offset_gui != 0.0
            or self.VehicleStates.steer_freeze_gui == 1
            or self.VehicleStates.steer_saturation_gui > 0.0
            or self.VehicleStates.steer_oscillation_gui != 0.0
            or self.VehicleStates.steer_random_gui == 1
            or self.VehicleStates.accelerate_offset_gui != 0.0
            or self.VehicleStates.accelerate_freeze_gui == 1
            or self.VehicleStates.accelerate_saturation_gui > 0.0
            or self.VehicleStates.accelerate_oscillation_gui != 0.0
            or self.VehicleStates.accelerate_runaway_gui == 1
        )

        if not fault_active:
            return

        #######################################################################
        # Hazard lights
        #######################################################################

        if self.VehicleStates.hl_gui == 1:

            if msg.lamp_cmd.l != 1 or msg.lamp_cmd.r != 1:

                msg.lamp_cmd.l = 1
                msg.lamp_cmd.r = 1

                modified = True

                self.logFault("HAZARD_LIGHTS", "ON")

        #######################################################################
        # Wheel lock
        #######################################################################

        if self.VehicleStates.wheel_gui == 0:

            if abs(self.VehicleStates.current_speed) < 0.1:

                if self.VehicleStates.steer_being_kept is None:

                    self.VehicleStates.steer_being_kept = msg.ctrl_cmd.steering_angle

                msg.ctrl_cmd.steering_angle = self.VehicleStates.steer_being_kept

                msg.ctrl_cmd.linear_velocity = 0.0

                modified = True

                self.logFault(
                    "WHEEL_LOCK",
                    self.VehicleStates.steer_being_kept,
                )

        else:

            self.VehicleStates.steer_being_kept = None

        #######################################################################
        # Steering offset
        #######################################################################

        if self.VehicleStates.steer_offset_gui != 0.0:

            msg.ctrl_cmd.steering_angle += self.VehicleStates.steer_offset_gui

            modified = True

            self.logFault(
                "STEERING_OFFSET",
                self.VehicleStates.steer_offset_gui,
            )

        #######################################################################
        # Steering freeze
        #######################################################################

        if self.VehicleStates.steer_freeze_gui == 1:

            if self.VehicleStates.steer_frozen_value is None:

                self.VehicleStates.steer_frozen_value = msg.ctrl_cmd.steering_angle

            msg.ctrl_cmd.steering_angle = self.VehicleStates.steer_frozen_value

            modified = True

            self.logFault(
                "STEERING_FREEZE",
                self.VehicleStates.steer_frozen_value,
            )

        else:

            self.VehicleStates.steer_frozen_value = None

        #######################################################################
        # Steering saturation
        #######################################################################

        if self.VehicleStates.steer_saturation_gui > 0:

            limit = self.VehicleStates.steer_saturation_gui
            saturated = False

            if msg.ctrl_cmd.steering_angle > limit:

                msg.ctrl_cmd.steering_angle = limit

                modified = True
                saturated = True

            elif msg.ctrl_cmd.steering_angle < -limit:

                msg.ctrl_cmd.steering_angle = -limit

                modified = True
                saturated = True

            if saturated:

                self.logFault(
                    "STEERING_SATURATION",
                    limit,
                )

        #######################################################################
        # Steering oscillation
        #######################################################################

        if self.VehicleStates.steer_oscillation_gui != 0.0:

            amplitude = self.VehicleStates.steer_oscillation_gui

            frequency = 2.0

            msg.ctrl_cmd.steering_angle += amplitude * math.sin(
                rospy.get_time() * frequency
            )

            modified = True

            self.logFault(
                "STEERING_OSCILLATION",
                amplitude,
            )

        #######################################################################
        # Steering random
        #######################################################################

        if self.VehicleStates.steer_random_gui == 1:

            msg.ctrl_cmd.steering_angle += random.uniform(
                -20.0,
                20.0,
            )

            modified = True

            self.logFault(
                "STEERING_RANDOM",
                msg.ctrl_cmd.steering_angle,
            )

        #######################################################################
        # Acceleration offset
        #######################################################################

        if self.VehicleStates.accelerate_offset_gui != 0.0:

            msg.ctrl_cmd.linear_acceleration += self.VehicleStates.accelerate_offset_gui
            msg.accel_cmd.accel += self.VehicleStates.accelerate_offset_gui
            modified = True

            self.logFault(
                "ACCELERATION_OFFSET",
                self.VehicleStates.accelerate_offset_gui,
            )

        #######################################################################
        # Acceleration freeze
        #######################################################################

        if self.VehicleStates.accelerate_freeze_gui == 1:

            if self.VehicleStates.accelerate_frozen_value is None:

                self.VehicleStates.accelerate_frozen_value = (
                    msg.ctrl_cmd.linear_acceleration
                )

            msg.ctrl_cmd.linear_acceleration = (
                self.VehicleStates.accelerate_frozen_value
            )
            msg.accel_cmd.accel = self.VehicleStates.accelerate_frozen_value
            modified = True

            self.logFault(
                "ACCELERATION_FREEZE",
                self.VehicleStates.accelerate_frozen_value,
            )

        else:

            self.VehicleStates.accelerate_frozen_value = None

        #######################################################################
        # Acceleration saturation
        #######################################################################

        if self.VehicleStates.accelerate_saturation_gui > 0:

            limit = self.VehicleStates.accelerate_saturation_gui

            if msg.ctrl_cmd.linear_acceleration > limit:

                msg.ctrl_cmd.linear_acceleration = limit
                msg.accel_cmd.accel = limit
                modified = True

                self.logFault(
                    "ACCELERATION_SATURATION",
                    limit,
                )

        #######################################################################
        # Runaway acceleration
        #######################################################################

        if self.VehicleStates.accelerate_runaway_gui == 1:

            msg.ctrl_cmd.linear_acceleration = 10.0
            msg.accel_cmd.accel = 10.0

            modified = True

            self.logFault(
                "ACCELERATION_RUNAWAY",
                10.0,
            )

        #######################################################################
        # Acceleration oscillation
        #######################################################################

        if self.VehicleStates.accelerate_oscillation_gui != 0.0:
            oscillation = self.VehicleStates.accelerate_oscillation_gui * math.sin(
                rospy.get_time()
            )

            msg.ctrl_cmd.linear_acceleration += oscillation
            msg.accel_cmd.accel += oscillation

            modified = True

            self.logFault(
                "ACCELERATION_OSCILLATION",
                self.VehicleStates.accelerate_oscillation_gui,
            )

        #######################################################################
        # Publish modified command
        #######################################################################

        if modified:
            with self._republishing_lock:
                self.republishing_vehicle_cmd = True
                try:
                    self.send_wheel_state_pub.publish(msg)
                finally:
                    self.republishing_vehicle_cmd = False

    def vehicleCmdFaultCallback(self, msg):
        """Maintain compatibility with the callback name used by manager.py."""
        self.vehicleCmdFaultInjectionCallback(msg)
