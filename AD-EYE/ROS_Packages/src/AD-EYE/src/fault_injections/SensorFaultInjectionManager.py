#!/usr/bin/env python

"""Inject bounded GNSS and LiDAR faults before Autoware consumes the data."""

import copy
import json
import math
import random
import struct
from collections import deque
import threading
import time

import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus, PointCloud2
from std_msgs.msg import String


METERS_PER_DEGREE_LATITUDE = 111111.0

# Set True to restore the configured min/max checks on fault input values.
ENFORCE_FAULT_INPUT_RANGES = False

# Change one axis per test, using the directions in the incoming LiDAR scan.
LIDAR_TRANSFORMS = {
    "LIDAR_TRANSLATE_X_M_command": ("translate", 0),
    "LIDAR_TRANSLATE_Y_M_command": ("translate", 1),
    "LIDAR_TRANSLATE_Z_M_command": ("translate", 2),
    "LIDAR_ROLL_DEG_command": ("rotate", 0),
    "LIDAR_PITCH_DEG_command": ("rotate", 1),
    "LIDAR_YAW_DEG_command": ("rotate", 2),
}


class SensorFaultInjectionManager:
    """Forward sensor messages while applying faults selected on vehicle_commands."""

    def __init__(self):
        self._lock = threading.Lock()
        self.recovery_hold = False
        self.recovery_token = ""
        self.availability = None

        self.enable_gnss = bool(rospy.get_param("~enable_gnss", True))
        self.enable_lidar = bool(rospy.get_param("~enable_lidar", True))

        self.gnss_east_bias_m = 0.0
        self.gnss_north_bias_m = 0.0
        self.gnss_dropout_enabled = False
        self.gnss_no_fix_enabled = False

        self.lidar_drop_every_n = 0
        self.lidar_timestamp_offset_s = 0.0
        self.lidar_message_count = 0
        self.lidar_freeze_mode = 0
        self.lidar_delay_s = 0.0
        self.lidar_sector_deg = 0.0
        self.lidar_range_m = 0.0
        self.lidar_point_loss_pct = 0.0
        self.lidar_transform_fault = ""
        self.lidar_transform_value = 0.0
        self.lidar_noise_stddev_m = 0.0
        self.lidar_keep_every_n = 0
        self.lidar_keep_count = 0
        self.point_random = random.Random()
        self.latest_lidar = None
        self.frozen_lidar = None
        self.delayed_lidar = deque()
        self.delayed_bytes = 0
        self.wall_clock = getattr(time, "monotonic", time.time)

        self.gnss_input_seen = False
        self.lidar_input_seen = False
        self.last_gnss_input_time = time.time()
        self.last_lidar_input_time = time.time()
        self.sensor_input_timeout_s = rospy.get_param(
            "~sensor_input_timeout_s", 2.0
        )
        self.max_gnss_bias_m = max(
            0.0, float(rospy.get_param("~max_gnss_bias_m", 5.0))
        )
        self.max_lidar_drop_every_n = max(
            1, int(rospy.get_param("~max_lidar_drop_every_n", 20))
        )
        self.max_lidar_timestamp_offset_s = max(
            0.0,
            float(rospy.get_param("~max_lidar_timestamp_offset_s", 0.2)),
        )
        self.fault_timeout_s = float(rospy.get_param("~fault_timeout_s", 5.0))
        self.max_fault_timeout_s = float(rospy.get_param("~max_fault_timeout_s", 300.0))
        self.active_fault = ""
        self.fault_deadline = None
        self.fault_ranges = {
            "GNSS_EAST_BIAS_M_command": [-self.max_gnss_bias_m, self.max_gnss_bias_m],
            "GNSS_NORTH_BIAS_M_command": [-self.max_gnss_bias_m, self.max_gnss_bias_m],
            "LIDAR_DROP_EVERY_N_command": [1.0, float(self.max_lidar_drop_every_n)],
            "LIDAR_TIMESTAMP_OFFSET_S_command": [-self.max_lidar_timestamp_offset_s, self.max_lidar_timestamp_offset_s],
            "LIDAR_FREEZE_command": [1.0, 2.0],
            "LIDAR_DELAY_S_command": [0.01, 5.0],
            "LIDAR_SECTOR_DEG_command": [1.0, 360.0],
            "LIDAR_RANGE_M_command": [0.1, 200.0],
            "LIDAR_POINT_LOSS_PCT_command": [1.0, 100.0],
            "LIDAR_NOISE_STDDEV_M_command": [0.001, 1.0],
            "LIDAR_KEEP_EVERY_N_command": [2.0, 20.0],
        }
        self.hard_fault_ranges = copy.deepcopy(self.fault_ranges)
        for fault, (kind, _axis) in LIDAR_TRANSFORMS.items():
            limits = [-5.0, 5.0] if kind == "translate" else [-180.0, 180.0]
            self.fault_ranges[fault] = limits[:]
            self.hard_fault_ranges[fault] = limits[:]

        gnss_input_topic = rospy.get_param("~gnss_input_topic", "/fix")
        gnss_output_topic = rospy.get_param(
            "~gnss_output_topic", "/adeye/fault_injection/fix"
        )
        lidar_input_topic = rospy.get_param(
            "~lidar_input_topic", "/os_cloud_node/points"
        )
        lidar_output_topic = rospy.get_param(
            "~lidar_output_topic", "/adeye/fault_injection/points"
        )
        self.lidar_input_topic = lidar_input_topic
        self.lidar_output_topic = lidar_output_topic

        self.gnss_pub = None
        self.lidar_pub = None
        self.lidar_timestamp_pub = None
        if self.enable_gnss:
            self.gnss_pub = rospy.Publisher(
                gnss_output_topic, NavSatFix, queue_size=10
            )
        if self.enable_lidar:
            self.lidar_pub = rospy.Publisher(
                lidar_output_topic, PointCloud2, queue_size=1
            )
            self.lidar_timestamp_pub = rospy.Publisher(
                "/adeye/lidar_timestamp_status", String, queue_size=1
            )

        self.status_pub = rospy.Publisher(
            "/adeye/physical_fault_status", String, queue_size=10
        )
        # Finish availability setup before any fault command can arrive.
        if rospy.get_param("/adeye/enable_fault_availability", False):
            import os
            import sys
            sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
            from fault_availability import SensorAvailability
            commands = list(self.fault_ranges) + ["GNSS_DROPOUT_command", "GNSS_NO_FIX_command"]
            commands = [command for command in commands if (self.enable_lidar if command.startswith("LIDAR") else self.enable_gnss)]
            self.availability = SensorAvailability(self, commands)
        self.command_sub = rospy.Subscriber(
            "/vehicle_commands", String, self.command_callback, queue_size=10
        )
        self.config_sub = rospy.Subscriber(
            "/adeye/physical_fault_config", String, self.config_callback, queue_size=10
        )
        self.gnss_sub = None
        self.lidar_sub = None
        if self.enable_gnss:
            self.gnss_sub = rospy.Subscriber(
                gnss_input_topic, NavSatFix, self.gnss_callback, queue_size=10
            )
        if self.enable_lidar:
            self.lidar_sub = rospy.Subscriber(
                lidar_input_topic, PointCloud2, self.lidar_callback, queue_size=1
            )
        self.health_timer = rospy.Timer(
            rospy.Duration(1.0), self.health_watchdog_callback
        )
        self.delay_stop = threading.Event()
        self.delay_worker = threading.Thread(target=self._delay_loop)
        self.delay_worker.daemon = True
        rospy.on_shutdown(self.delay_stop.set)
        self.delay_worker.start()
        if rospy.get_param("/adeye/enable_manual_recovery", False):
            self.recovery_token = rospy.get_param("/adeye/vehicle_recovery_token", "")
            self.recovery_hold = bool(self.recovery_token)
            self.recovery_ack = rospy.Publisher("/adeye/vehicle_recovery_ack", String, queue_size=10, latch=True)
            self.recovery_sub = rospy.Subscriber("/adeye/vehicle_recovery_control", String, self.recovery_callback, queue_size=10)

        rospy.loginfo(
            "Sensor fault injector: GNSS %s, LiDAR %s",
            "%s -> %s" % (gnss_input_topic, gnss_output_topic)
            if self.enable_gnss else "disabled",
            "%s -> %s" % (lidar_input_topic, lidar_output_topic)
            if self.enable_lidar else "disabled",
        )

    def recovery_callback(self, message):
        action, separator, token = message.data.partition(":")
        if not separator or not token or any(c not in "0123456789abcdef" for c in token):
            return
        with self._lock:
            if action == "PREPARE":
                self.recovery_hold = True
                cancelled = self.active_fault
                self._clear_sensor_faults()
                self.lidar_message_count = 0
                self.recovery_token = token
                if cancelled:
                    self._publish_status("reset", cancelled, False, 0.0, "cancelled for manual software reset")
                phase = "prepared"
            elif action == "RELEASE" and token == self.recovery_token:
                self.recovery_hold = False
                phase = "released"
            else:
                return
            self.recovery_ack.publish(String(data=json.dumps({"component": rospy.get_name(), "token": token, "phase": phase})))

    def health_watchdog_callback(self, _event):
        """Report missing sensor data without reporting an applied fault as a problem."""

        now = time.time()
        with self._lock:
            if self.availability:
                self.availability.publish()
            if self.fault_deadline is not None and now >= self.fault_deadline:
                expired_fault = self.active_fault
                self._clear_sensor_faults()
                self._publish_status("timeout", expired_fault, False, 0.0, "sensor fault timeout expired")
            elif self.active_fault:
                self._publish_status(
                    "active", self.active_fault, True,
                    self._active_fault_value(), "sensor fault is active"
                )
            gnss_seen = self.gnss_input_seen
            lidar_seen = self.lidar_input_seen
            gnss_age = now - self.last_gnss_input_time
            lidar_age = now - self.last_lidar_input_time

        if self.enable_gnss and gnss_age > self.sensor_input_timeout_s:
            rospy.logwarn_throttle(
                5.0, "GNSS data has %s for %.1f s; diagnostic only",
                "not arrived" if not gnss_seen else "not updated", gnss_age,
            )
        if self.enable_lidar and lidar_age > self.sensor_input_timeout_s:
            rospy.logwarn_throttle(
                5.0, "LiDAR data has %s for %.1f s; diagnostic only",
                "not arrived" if not lidar_seen else "not updated", lidar_age,
            )

    def _publish_status(self, event, fault, active, value, detail, limits=None, hard_limits=None):
        remaining = max(0.0, self.fault_deadline - time.time()) if self.fault_deadline is not None else 0.0
        status = {
            "source": "sensor", "event": event, "fault": fault,
            "active": active, "value": value, "timeout_s": self.fault_timeout_s,
            "remaining_s": remaining, "detail": detail,
        }
        if limits is not None:
            status["min_value"], status["max_value"] = limits
        if hard_limits is not None:
            status["hard_min_value"], status["hard_max_value"] = hard_limits
        self.status_pub.publish(String(data=json.dumps(status)))

    def _clear_sensor_faults(self):
        self.gnss_east_bias_m = 0.0
        self.gnss_north_bias_m = 0.0
        self.gnss_dropout_enabled = False
        self.gnss_no_fix_enabled = False
        self.lidar_drop_every_n = 0
        self.lidar_timestamp_offset_s = 0.0
        self.lidar_freeze_mode = 0
        self.lidar_delay_s = 0.0
        self.lidar_sector_deg = 0.0
        self.lidar_range_m = 0.0
        self.lidar_point_loss_pct = 0.0
        self.lidar_transform_fault = ""
        self.lidar_transform_value = 0.0
        self.lidar_noise_stddev_m = 0.0
        self.lidar_keep_every_n = 0
        self.lidar_keep_count = 0
        self.frozen_lidar = None
        self.delayed_lidar.clear()
        self.delayed_bytes = 0
        self.active_fault = ""
        self.fault_deadline = None

    def _active_fault_value(self):
        values = {
            "GNSS_EAST_BIAS_M_command": self.gnss_east_bias_m,
            "GNSS_NORTH_BIAS_M_command": self.gnss_north_bias_m,
            "GNSS_DROPOUT_command": 1.0 if self.gnss_dropout_enabled else 0.0,
            "GNSS_NO_FIX_command": 1.0 if self.gnss_no_fix_enabled else 0.0,
            "LIDAR_DROP_EVERY_N_command": float(self.lidar_drop_every_n),
            "LIDAR_TIMESTAMP_OFFSET_S_command": self.lidar_timestamp_offset_s,
            "LIDAR_FREEZE_command": float(self.lidar_freeze_mode),
            "LIDAR_DELAY_S_command": self.lidar_delay_s,
            "LIDAR_SECTOR_DEG_command": self.lidar_sector_deg,
            "LIDAR_RANGE_M_command": self.lidar_range_m,
            "LIDAR_POINT_LOSS_PCT_command": self.lidar_point_loss_pct,
            "LIDAR_NOISE_STDDEV_M_command": self.lidar_noise_stddev_m,
            "LIDAR_KEEP_EVERY_N_command": float(self.lidar_keep_every_n),
        }
        if self.active_fault in LIDAR_TRANSFORMS:
            return self.lidar_transform_value
        return values.get(self.active_fault, 0.0)

    def _can_change_fault(self, fault, value):
        if value == 0 or not self.active_fault or self.active_fault == fault:
            return True
        self._publish_status(
            "rejected", fault, False, value,
            "another sensor fault is active"
        )
        return False

    def _fault_changed(self, fault, value):
        if value == 0:
            if self.active_fault == fault:
                self.active_fault = ""
                self.fault_deadline = None
            self._publish_status("reset", fault, False, value, "sensor fault disabled")
        else:
            self.active_fault = fault
            self.fault_deadline = time.time() + self.fault_timeout_s
            self._publish_status("active", fault, True, value, "sensor fault applied")

    def config_callback(self, message):
        command = message.data
        with self._lock:
            if command == "FAULT_LIMITS_QUERY_command":
                for fault, limits in self.fault_ranges.items():
                    self._publish_status("limits", fault, False, 0.0, "sensor fault ranges", limits, self.hard_fault_ranges[fault])
                return

            timeout_prefix = "PHYSICAL_FAULT_TIMEOUT_command="
            if command.startswith(timeout_prefix):
                value = self._command_value(command, timeout_prefix)
                if value is None or value <= 0 or value > self.max_fault_timeout_s:
                    self._publish_status("rejected", "PHYSICAL_FAULT_TIMEOUT_command", False, 0.0, "timeout is outside the allowed range")
                elif self.active_fault:
                    self._publish_status("rejected", "PHYSICAL_FAULT_TIMEOUT_command", False, value, "cannot change timeout while a fault is active")
                else:
                    self.fault_timeout_s = value
                    self._publish_status("configured", "PHYSICAL_FAULT_TIMEOUT_command", False, value, "sensor fault timeout updated")
                return

            prefix = "FAULT_LIMITS_command="
            if not command.startswith(prefix):
                return
            parts = command[len(prefix):].split(",")
            if len(parts) != 3 or parts[0] not in self.fault_ranges:
                return
            try:
                minimum, maximum = float(parts[1]), float(parts[2])
            except ValueError:
                return
            fault = parts[0]
            hard_minimum, hard_maximum = self.hard_fault_ranges[fault]
            usable_numbers = (
                not math.isnan(minimum) and not math.isinf(minimum)
                and not math.isnan(maximum) and not math.isinf(maximum)
            )
            whole_number_range = (
                fault not in ("LIDAR_DROP_EVERY_N_command", "LIDAR_KEEP_EVERY_N_command")
                or (usable_numbers and minimum == int(minimum) and maximum == int(maximum))
            )
            if self.active_fault:
                self._publish_status("rejected", fault, False, 0.0, "cannot change the range while a fault is active")
            elif not (
                usable_numbers
                and hard_minimum <= minimum < maximum <= hard_maximum
                and whole_number_range
            ):
                self._publish_status("rejected", fault, False, 0.0, "requested range is outside the allowed safety range")
            else:
                self.fault_ranges[fault] = [minimum, maximum]
                self._publish_status("configured", fault, False, 0.0, "sensor fault range updated", [minimum, maximum])

    def command_callback(self, message):
        command = message.data

        with self._lock:
            denied = self.availability.blocked_activation(command) if self.availability else ""
            if denied:
                self._publish_status("rejected", denied, False, 0, "fault disabled by Test Configuration")
                return
            if self.recovery_hold:
                return
            for fault, attribute in [("LIDAR_FREEZE_command", "lidar_freeze_mode"),
                                     ("LIDAR_DELAY_S_command", "lidar_delay_s"),
                                     ("LIDAR_SECTOR_DEG_command", "lidar_sector_deg"),
                                     ("LIDAR_RANGE_M_command", "lidar_range_m"),
                                     ("LIDAR_NOISE_STDDEV_M_command", "lidar_noise_stddev_m"),
                                     ("LIDAR_KEEP_EVERY_N_command", "lidar_keep_every_n"),
                                     ("LIDAR_POINT_LOSS_PCT_command", "lidar_point_loss_pct")] + [
                                     (fault, "lidar_transform_value") for fault in LIDAR_TRANSFORMS]:
                value = self._command_value(command, fault + "=")
                if value is None:
                    continue
                # These checks protect valid modes, scan shapes and memory use; they are not car safety limits.
                maximum = {"LIDAR_DELAY_S_command": 5, "LIDAR_SECTOR_DEG_command": 360,
                           "LIDAR_POINT_LOSS_PCT_command": 100}.get(fault)
                valid = (value in (0, 1, 2) if fault == "LIDAR_FREEZE_command" else
                         value >= 0 and (maximum is None or value <= maximum))
                if fault in LIDAR_TRANSFORMS:
                    valid = True  # Invalid numbers are already rejected. An offset may be positive or negative.
                if fault == "LIDAR_KEEP_EVERY_N_command":
                    valid = value == 0 or (2 <= value <= 9007199254740991 and value == int(value))
                if ENFORCE_FAULT_INPUT_RANGES and value != 0:
                    minimum, maximum = self.fault_ranges[fault]
                    valid = valid and minimum <= value <= maximum
                if not self.enable_lidar or not valid:
                    self._publish_status("rejected", fault, False, value, "LiDAR disabled or value outside the supported range")
                elif self._can_change_fault(fault, value):
                    if value and fault == "LIDAR_FREEZE_command" and self.latest_lidar is None:
                        self._publish_status("rejected", fault, False, value, "wait for a LiDAR scan before freezing")
                        return
                    if value == 0:
                        if self.active_fault == fault:
                            self._clear_sensor_faults()
                        self._publish_status("reset", fault, False, 0, "sensor fault disabled; buffered scans discarded")
                    else:
                        self.delayed_lidar.clear()
                        self.delayed_bytes = 0
                        if fault == "LIDAR_FREEZE_command":
                            self.frozen_lidar = copy.deepcopy(self.latest_lidar)
                        setattr(self, attribute, value)
                        if fault == "LIDAR_KEEP_EVERY_N_command":
                            self.lidar_keep_every_n = int(value)
                            self.lidar_keep_count = 0
                        if fault in LIDAR_TRANSFORMS:
                            self.lidar_transform_fault = fault
                        self._fault_changed(fault, value)
                return
            value = self._command_value(command, "GNSS_EAST_BIAS_M_command=")
            if value is not None:
                if not self.enable_gnss:
                    self._publish_status("rejected", "GNSS_EAST_BIAS_M_command", False, value, "GNSS proxy is disabled")
                elif not self._can_change_fault("GNSS_EAST_BIAS_M_command", value):
                    pass
                elif not ENFORCE_FAULT_INPUT_RANGES or self.fault_ranges["GNSS_EAST_BIAS_M_command"][0] <= value <= self.fault_ranges["GNSS_EAST_BIAS_M_command"][1]:
                    self.gnss_east_bias_m = value
                    self._fault_changed("GNSS_EAST_BIAS_M_command", value)
                else:
                    rospy.logwarn("GNSS east offset rejected because it is outside the test limit: %s", value)

            value = self._command_value(command, "GNSS_NORTH_BIAS_M_command=")
            if value is not None:
                if not self.enable_gnss:
                    self._publish_status("rejected", "GNSS_NORTH_BIAS_M_command", False, value, "GNSS proxy is disabled")
                elif not self._can_change_fault("GNSS_NORTH_BIAS_M_command", value):
                    pass
                elif not ENFORCE_FAULT_INPUT_RANGES or self.fault_ranges["GNSS_NORTH_BIAS_M_command"][0] <= value <= self.fault_ranges["GNSS_NORTH_BIAS_M_command"][1]:
                    self.gnss_north_bias_m = value
                    self._fault_changed("GNSS_NORTH_BIAS_M_command", value)
                else:
                    rospy.logwarn("GNSS north offset rejected because it is outside the test limit: %s", value)

            if command == "GNSS_DROPOUT_command=1":
                if self.enable_gnss and self._can_change_fault("GNSS_DROPOUT_command", 1):
                    self.gnss_dropout_enabled = True
                    self._fault_changed("GNSS_DROPOUT_command", 1)
            elif command == "GNSS_DROPOUT_command=0":
                self.gnss_dropout_enabled = False
                self._fault_changed("GNSS_DROPOUT_command", 0)

            if command == "GNSS_NO_FIX_command=1":
                if self.enable_gnss and self._can_change_fault("GNSS_NO_FIX_command", 1):
                    self.gnss_no_fix_enabled = True
                    self._fault_changed("GNSS_NO_FIX_command", 1)
            elif command == "GNSS_NO_FIX_command=0":
                self.gnss_no_fix_enabled = False
                self._fault_changed("GNSS_NO_FIX_command", 0)

            value = self._command_value(command, "LIDAR_DROP_EVERY_N_command=")
            if value is not None:
                integer_value = int(value)
                if not self.enable_lidar:
                    self._publish_status("rejected", "LIDAR_DROP_EVERY_N_command", False, value, "LiDAR proxy is disabled")
                elif not self._can_change_fault("LIDAR_DROP_EVERY_N_command", integer_value):
                    pass
                elif value != integer_value or integer_value < 0:
                    rospy.logwarn("LiDAR drop interval rejected because it is not a whole number: %s", value)
                elif not ENFORCE_FAULT_INPUT_RANGES or integer_value == 0 or self.fault_ranges["LIDAR_DROP_EVERY_N_command"][0] <= integer_value <= self.fault_ranges["LIDAR_DROP_EVERY_N_command"][1]:
                    if integer_value > 0 and self.lidar_drop_every_n == 0:
                        self.lidar_message_count = 0
                    self.lidar_drop_every_n = integer_value
                    self._fault_changed("LIDAR_DROP_EVERY_N_command", integer_value)
                else:
                    rospy.logwarn("LiDAR drop interval rejected because it is outside the test limit: %s", value)

            value = self._command_value(
                command, "LIDAR_TIMESTAMP_OFFSET_S_command="
            )
            if value is not None:
                if not self.enable_lidar:
                    self._publish_status("rejected", "LIDAR_TIMESTAMP_OFFSET_S_command", False, value, "LiDAR proxy is disabled")
                elif not self._can_change_fault("LIDAR_TIMESTAMP_OFFSET_S_command", value):
                    pass
                elif not ENFORCE_FAULT_INPUT_RANGES or self.fault_ranges["LIDAR_TIMESTAMP_OFFSET_S_command"][0] <= value <= self.fault_ranges["LIDAR_TIMESTAMP_OFFSET_S_command"][1]:
                    self.lidar_timestamp_offset_s = value
                    self._fault_changed("LIDAR_TIMESTAMP_OFFSET_S_command", value)
                else:
                    rospy.logwarn("LiDAR time offset rejected because it is outside the test limit: %s", value)

    @staticmethod
    def _command_value(command, prefix):
        if not command.startswith(prefix):
            return None

        try:
            value = float(command[len(prefix):])
        except ValueError:
            rospy.logwarn("Invalid sensor-fault command: %s", command)
            return None

        if math.isnan(value) or math.isinf(value):
            rospy.logwarn("Sensor fault command does not contain a usable number: %s", command)
            return None

        return value

    def gnss_callback(self, message):
        with self._lock:
            self.gnss_input_seen = True
            self.last_gnss_input_time = time.time()
            dropout_enabled = self.gnss_dropout_enabled
            no_fix_enabled = self.gnss_no_fix_enabled
            east_bias_m = self.gnss_east_bias_m
            north_bias_m = self.gnss_north_bias_m

        if dropout_enabled:
            rospy.logwarn_throttle(1.0, "GNSS fault: dropping fixes.")
            return

        faulty_message = copy.deepcopy(message)

        if no_fix_enabled:
            faulty_message.status.status = NavSatStatus.STATUS_NO_FIX

        faulty_message.latitude += north_bias_m / METERS_PER_DEGREE_LATITUDE

        meters_per_degree_longitude = (
            METERS_PER_DEGREE_LATITUDE
            * math.cos(math.radians(faulty_message.latitude))
        )
        if abs(meters_per_degree_longitude) > 1e-6:
            faulty_message.longitude += east_bias_m / meters_per_degree_longitude

        self.gnss_pub.publish(faulty_message)

    def lidar_callback(self, message):
        with self._lock:
            self._expire_lidar_fault()
            self.lidar_input_seen = True
            self.last_lidar_input_time = time.time()
            self.latest_lidar = message
            self.lidar_message_count += 1
            message_count = self.lidar_message_count
            drop_every_n = self.lidar_drop_every_n
            timestamp_offset_s = self.lidar_timestamp_offset_s
            if self.lidar_keep_every_n:
                self.lidar_keep_count += 1
                if self.lidar_keep_count % self.lidar_keep_every_n != 0:
                    return
            if drop_every_n > 0 and message_count % drop_every_n == 0:
                return
            if self.lidar_delay_s > 0:
                # Stop the fault before it fills memory or sends many delayed scans at once.
                if len(self.delayed_lidar) >= 100 or self.delayed_bytes + len(message.data) > 64 * 1024 * 1024:
                    fault = self.active_fault
                    self._clear_sensor_faults()
                    self._publish_status("reset", fault, False, 0, "delay buffer full; fault stopped and buffered scans discarded")
                else:
                    self.delayed_lidar.append((self.wall_clock() + self.lidar_delay_s, message))
                    self.delayed_bytes += len(message.data)
                    return
            faulty_message = message
            if self.lidar_freeze_mode:
                faulty_message = copy.copy(self.frozen_lidar)
                faulty_message.header = copy.deepcopy(self.frozen_lidar.header)
                if self.lidar_freeze_mode == 2:
                    faulty_message.header.stamp = message.header.stamp
            if self.lidar_sector_deg or self.lidar_range_m or self.lidar_point_loss_pct or self.lidar_transform_fault or self.lidar_noise_stddev_m:
                try:
                    if self.lidar_noise_stddev_m:
                        faulty_message = self._transform_cloud(message, "noise", 0, self.lidar_noise_stddev_m, self.point_random)
                    elif self.lidar_transform_fault:
                        kind, axis = LIDAR_TRANSFORMS[self.lidar_transform_fault]
                        faulty_message = self._transform_cloud(message, kind, axis, self.lidar_transform_value)
                    elif self.lidar_sector_deg:
                        faulty_message = self._remove_sector(message, self.lidar_sector_deg)
                    elif self.lidar_range_m:
                        faulty_message = self._limit_range(message, self.lidar_range_m)
                    else:
                        probability = self.lidar_point_loss_pct / 100.0
                        faulty_message = self._filter_points(message, lambda _start: self.point_random.random() >= probability)
                except (ValueError, struct.error, OverflowError) as error:
                    fault = self.active_fault
                    self._clear_sensor_faults()
                    self._publish_status("reset", fault, False, 0, "unsupported cloud; fault stopped: " + str(error))
                    faulty_message = message
            self._publish_lidar(message, faulty_message, timestamp_offset_s)

    def _expire_lidar_fault(self):
        if self.fault_deadline is not None and time.time() >= self.fault_deadline:
            fault = self.active_fault
            self._clear_sensor_faults()
            self._publish_status("timeout", fault, False, 0, "sensor fault timeout expired; buffered scans discarded")

    def _delay_loop(self):
        while not self.delay_stop.wait(0.01):
            with self._lock:
                self._drain_delay()

    def _drain_delay(self):
        self._expire_lidar_fault()
        now = self.wall_clock()
        while self.delayed_lidar and self.delayed_lidar[0][0] <= now:
            _, message = self.delayed_lidar.popleft()
            self.delayed_bytes -= len(message.data)
            self._publish_lidar(message, message, 0.0)

    @staticmethod
    def _transform_cloud(message, kind, axis, value, noise_random=None):
        fields = {field.name: field for field in message.fields}
        names = ("x", "y", "z")
        if not all(name in fields and fields[name].datatype in (7, 8) and fields[name].count == 1 for name in names):
            raise ValueError("x/y/z must be FLOAT32 or FLOAT64")
        if message.point_step <= 0 or message.row_step < message.width * message.point_step or len(message.data) < message.height * message.row_step:
            raise ValueError("invalid cloud layout")
        endian = ">" if message.is_bigendian else "<"
        readers = [struct.Struct(endian + ("f" if fields[name].datatype == 7 else "d")) for name in names]
        spans = []
        for name, reader in zip(names, readers):
            offset = fields[name].offset
            if offset < 0 or offset + reader.size > message.point_step:
                raise ValueError("invalid coordinate offset")
            spans.append((offset, offset + reader.size))
        if any(max(a[0], b[0]) < min(a[1], b[1]) for index, a in enumerate(spans) for b in spans[index + 1:]):
            raise ValueError("overlapping coordinate fields")
        data = bytearray(message.data)
        # Simplify the angle first so very large values do not cause rounding errors.
        angle = math.radians(value % 360) if kind == "rotate" else 0.0
        cosine, sine = math.cos(angle), math.sin(angle)
        for row in range(message.height):
            for column in range(message.width):
                start = row * message.row_step + column * message.point_step
                coordinates = [reader.unpack_from(message.data, start + fields[name].offset)[0] for name, reader in zip(names, readers)]
                if any(math.isnan(number) or math.isinf(number) for number in coordinates):
                    continue  # Keep invalid points exactly as received.
                if kind == "noise":
                    coordinates = [number + noise_random.gauss(0.0, value) for number in coordinates]
                elif kind == "translate":
                    coordinates[axis] += value
                else:
                    # Rotate around +X for roll, +Y for pitch, or +Z for yaw.
                    # Positive angles follow the right-hand rule.
                    first, second = ((1, 2), (2, 0), (0, 1))[axis]
                    a, b = coordinates[first], coordinates[second]
                    coordinates[first], coordinates[second] = cosine * a - sine * b, sine * a + cosine * b
                if any(math.isnan(number) or math.isinf(number) for number in coordinates):
                    raise ValueError("transformed coordinate is not finite")
                for name, reader, number in zip(names, readers, coordinates):
                    reader.pack_into(data, start + fields[name].offset, number)
        output = copy.copy(message)
        output.data = bytes(data)
        return output

    @staticmethod
    def _filter_points(message, keep):
        if message.point_step <= 0 or message.row_step < message.width * message.point_step or len(message.data) < message.height * message.row_step:
            raise ValueError("invalid cloud layout")
        kept = []
        for row in range(message.height):
            for column in range(message.width):
                start = row * message.row_step + column * message.point_step
                if keep(start):
                    kept.append(message.data[start:start + message.point_step])
        result = copy.copy(message)
        result.height = 1
        result.width = len(kept)
        result.row_step = result.width * result.point_step
        result.data = b"".join(kept)
        return result

    @staticmethod
    def _limit_range(message, maximum_m):
        fields = {field.name: field for field in message.fields}
        names = ("x", "y", "z")
        if not all(name in fields and fields[name].datatype in (7, 8) and fields[name].count == 1 for name in names):
            raise ValueError("x/y/z must be FLOAT32 or FLOAT64")
        endian = ">" if message.is_bigendian else "<"
        readers = [struct.Struct(endian + ("f" if fields[name].datatype == 7 else "d")) for name in names]
        for name, reader in zip(names, readers):
            if fields[name].offset < 0 or fields[name].offset + reader.size > message.point_step:
                raise ValueError("invalid coordinate offset")
        def keep(start):
            values = [reader.unpack_from(message.data, start + fields[name].offset)[0] for name, reader in zip(names, readers)]
            # Invalid points do not describe a usable distance.
            return (all(not math.isnan(value) and not math.isinf(value) for value in values)
                    and math.hypot(math.hypot(values[0], values[1]), values[2]) <= maximum_m)
        return SensorFaultInjectionManager._filter_points(message, keep)

    @staticmethod
    def _remove_sector(message, width_deg):
        fields = {field.name: field for field in message.fields}
        if not all(name in fields and fields[name].datatype in (7, 8) and fields[name].count == 1 for name in ("x", "y")):
            raise ValueError("x/y must be FLOAT32 or FLOAT64")
        endian = ">" if message.is_bigendian else "<"
        readers = [struct.Struct(endian + ("f" if fields[name].datatype == 7 else "d")) for name in ("x", "y")]
        if message.point_step <= 0 or message.row_step < message.width * message.point_step or len(message.data) < message.height * message.row_step:
            raise ValueError("invalid cloud layout")
        for name, reader in zip(("x", "y"), readers):
            if fields[name].offset < 0 or fields[name].offset + reader.size > message.point_step:
                raise ValueError("invalid coordinate offset")
        kept = []
        for row in range(message.height):
            for column in range(message.width):
                start = row * message.row_step + column * message.point_step
                x, y = [reader.unpack_from(message.data, start + fields[name].offset)[0] for name, reader in zip(("x", "y"), readers)]
                if math.isnan(x) or math.isnan(y) or abs(math.atan2(y, x)) > math.radians(width_deg) / 2:
                    kept.append(message.data[start:start + message.point_step])
        result = copy.copy(message)
        result.height = 1
        result.width = len(kept)
        result.row_step = result.width * result.point_step
        result.data = b"".join(kept)
        return result

    def _publish_lidar(self, message, output, timestamp_offset_s):

        # Keep the selected point data; copy the header before changing its time.
        faulty_message = copy.copy(output)
        faulty_message.header = copy.deepcopy(output.header)
        faulty_message.header.stamp += rospy.Duration(timestamp_offset_s)

        self.lidar_pub.publish(faulty_message)
        # Compare this scan's time before and after the change, not two different scans.
        timestamp_status = {
            "input_topic": self.lidar_input_topic,
            "output_topic": self.lidar_output_topic,
            "sequence": int(message.header.seq),
            "frame_id": message.header.frame_id,
            "input_stamp": {"secs": message.header.stamp.secs,
                            "nsecs": message.header.stamp.nsecs},
            "output_stamp": {"secs": faulty_message.header.stamp.secs,
                             "nsecs": faulty_message.header.stamp.nsecs},
            "requested_offset_s": timestamp_offset_s,
            "measured_offset_s": (faulty_message.header.stamp - message.header.stamp).to_sec(),
        }
        self.lidar_timestamp_pub.publish(String(data=json.dumps(timestamp_status, allow_nan=False)))


if __name__ == "__main__":
    rospy.init_node("sensor_fault_injection_manager")
    SensorFaultInjectionManager()
    rospy.spin()
