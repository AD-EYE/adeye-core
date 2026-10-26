#!/usr/bin/env python

"""Inject bounded GNSS and LiDAR faults before Autoware consumes the data."""

import copy
import json
import math
import threading
import time

import rospy
from sensor_msgs.msg import NavSatFix, NavSatStatus, PointCloud2
from std_msgs.msg import String


METERS_PER_DEGREE_LATITUDE = 111111.0


class SensorFaultInjectionManager:
    """Forward sensor messages while applying faults selected on vehicle_commands."""

    def __init__(self):
        self._lock = threading.Lock()

        self.enable_gnss = bool(rospy.get_param("~enable_gnss", True))
        self.enable_lidar = bool(rospy.get_param("~enable_lidar", True))

        self.gnss_east_bias_m = 0.0
        self.gnss_north_bias_m = 0.0
        self.gnss_dropout_enabled = False
        self.gnss_no_fix_enabled = False

        self.lidar_drop_every_n = 0
        self.lidar_timestamp_offset_s = 0.0
        self.lidar_message_count = 0

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
        }
        self.hard_fault_ranges = copy.deepcopy(self.fault_ranges)

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

        self.gnss_pub = None
        self.lidar_pub = None
        if self.enable_gnss:
            self.gnss_pub = rospy.Publisher(
                gnss_output_topic, NavSatFix, queue_size=10
            )
        if self.enable_lidar:
            self.lidar_pub = rospy.Publisher(
                lidar_output_topic, PointCloud2, queue_size=1
            )

        self.command_sub = rospy.Subscriber(
            "/vehicle_commands", String, self.command_callback, queue_size=10
        )
        self.status_pub = rospy.Publisher(
            "/adeye/physical_fault_status", String, queue_size=10
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

        rospy.loginfo(
            "Sensor fault injector: GNSS %s, LiDAR %s",
            "%s -> %s" % (gnss_input_topic, gnss_output_topic)
            if self.enable_gnss else "disabled",
            "%s -> %s" % (lidar_input_topic, lidar_output_topic)
            if self.enable_lidar else "disabled",
        )

    def health_watchdog_callback(self, _event):
        """Report missing sensor data without reporting an applied fault as a problem."""

        now = time.time()
        with self._lock:
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
            rospy.logerr_throttle(
                5.0, "GNSS data has %s for %.1f s",
                "not arrived" if not gnss_seen else "not updated", gnss_age,
            )
        if self.enable_lidar and lidar_age > self.sensor_input_timeout_s:
            rospy.logerr_throttle(
                5.0, "LiDAR data has %s for %.1f s",
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
        }
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
                fault != "LIDAR_DROP_EVERY_N_command"
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
            value = self._command_value(command, "GNSS_EAST_BIAS_M_command=")
            if value is not None:
                if not self.enable_gnss:
                    self._publish_status("rejected", "GNSS_EAST_BIAS_M_command", False, value, "GNSS proxy is disabled")
                elif not self._can_change_fault("GNSS_EAST_BIAS_M_command", value):
                    pass
                elif self.fault_ranges["GNSS_EAST_BIAS_M_command"][0] <= value <= self.fault_ranges["GNSS_EAST_BIAS_M_command"][1]:
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
                elif self.fault_ranges["GNSS_NORTH_BIAS_M_command"][0] <= value <= self.fault_ranges["GNSS_NORTH_BIAS_M_command"][1]:
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
                elif value != integer_value:
                    rospy.logwarn("LiDAR drop interval rejected because it is not a whole number: %s", value)
                elif integer_value == 0 or self.fault_ranges["LIDAR_DROP_EVERY_N_command"][0] <= integer_value <= self.fault_ranges["LIDAR_DROP_EVERY_N_command"][1]:
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
                elif self.fault_ranges["LIDAR_TIMESTAMP_OFFSET_S_command"][0] <= value <= self.fault_ranges["LIDAR_TIMESTAMP_OFFSET_S_command"][1]:
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
            self.lidar_input_seen = True
            self.last_lidar_input_time = time.time()
            self.lidar_message_count += 1
            message_count = self.lidar_message_count
            drop_every_n = self.lidar_drop_every_n
            timestamp_offset_s = self.lidar_timestamp_offset_s

        if drop_every_n > 0 and message_count % drop_every_n == 0:
            rospy.logwarn_throttle(1.0, "LiDAR fault: dropping scan.")
            return

        # Point data is unchanged; only the timestamp is modified.
        faulty_message = copy.copy(message)
        faulty_message.header = copy.copy(message.header)
        faulty_message.header.stamp += rospy.Duration(timestamp_offset_s)

        self.lidar_pub.publish(faulty_message)


if __name__ == "__main__":
    rospy.init_node("sensor_fault_injection_manager")
    SensorFaultInjectionManager()
    rospy.spin()
