#!/usr/bin/env python

"""Inject bounded GNSS and LiDAR faults before Autoware consumes the data."""

import copy
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
                lidar_output_topic, PointCloud2, queue_size=10
            )

        self.command_sub = rospy.Subscriber(
            "/vehicle_commands", String, self.command_callback, queue_size=10
        )
        self.gnss_sub = None
        self.lidar_sub = None
        if self.enable_gnss:
            self.gnss_sub = rospy.Subscriber(
                gnss_input_topic, NavSatFix, self.gnss_callback, queue_size=10
            )
        if self.enable_lidar:
            self.lidar_sub = rospy.Subscriber(
                lidar_input_topic, PointCloud2, self.lidar_callback, queue_size=10
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
            gnss_seen = self.gnss_input_seen
            lidar_seen = self.lidar_input_seen
            gnss_age = now - self.last_gnss_input_time
            lidar_age = now - self.last_lidar_input_time

        if self.enable_gnss and gnss_age > self.sensor_input_timeout_s:
            rospy.logerr_throttle(
                5.0,
                "GNSS data has %s for %.1f s",
                "not arrived" if not gnss_seen else "not updated",
                gnss_age,
            )

        if self.enable_lidar and lidar_age > self.sensor_input_timeout_s:
            rospy.logerr_throttle(
                5.0,
                "LiDAR data has %s for %.1f s",
                "not arrived" if not lidar_seen else "not updated",
                lidar_age,
            )

    def command_callback(self, message):
        command = message.data

        with self._lock:
            value = self._command_value(command, "GNSS_EAST_BIAS_M_command=")
            if value is not None:
                if abs(value) <= self.max_gnss_bias_m:
                    self.gnss_east_bias_m = value
                else:
                    rospy.logwarn("GNSS east offset rejected because it is outside the test limit: %s", value)

            value = self._command_value(command, "GNSS_NORTH_BIAS_M_command=")
            if value is not None:
                if abs(value) <= self.max_gnss_bias_m:
                    self.gnss_north_bias_m = value
                else:
                    rospy.logwarn("GNSS north offset rejected because it is outside the test limit: %s", value)

            if command == "GNSS_DROPOUT_command=1":
                self.gnss_dropout_enabled = True
            elif command == "GNSS_DROPOUT_command=0":
                self.gnss_dropout_enabled = False

            if command == "GNSS_NO_FIX_command=1":
                self.gnss_no_fix_enabled = True
            elif command == "GNSS_NO_FIX_command=0":
                self.gnss_no_fix_enabled = False

            value = self._command_value(command, "LIDAR_DROP_EVERY_N_command=")
            if value is not None:
                integer_value = int(value)
                if value != integer_value:
                    rospy.logwarn("LiDAR drop interval rejected because it is not a whole number: %s", value)
                elif integer_value == 0 or 1 <= integer_value <= self.max_lidar_drop_every_n:
                    self.lidar_drop_every_n = integer_value
                else:
                    rospy.logwarn("LiDAR drop interval rejected because it is outside the test limit: %s", value)

            value = self._command_value(
                command, "LIDAR_TIMESTAMP_OFFSET_S_command="
            )
            if value is not None:
                if abs(value) <= self.max_lidar_timestamp_offset_s:
                    self.lidar_timestamp_offset_s = value
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
