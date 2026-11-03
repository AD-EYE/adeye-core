"""Test the real sensor message handler using simple copies of ROS message types."""
import json
import random
import struct
import time
import os
import sys
import types
import unittest


class Obj(object):
    def __init__(self, **values):
        self.__dict__.update(values)


class Duration(object):
    def __init__(self, seconds=0):
        self.nanoseconds = int(round(seconds * 1000000000))

    def to_sec(self):
        return self.nanoseconds / 1000000000.0


class Stamp(object):
    def __init__(self, secs, nsecs=0):
        self.secs = secs + nsecs // 1000000000
        self.nsecs = nsecs % 1000000000

    def __add__(self, duration):
        return Stamp(self.secs, self.nsecs + duration.nanoseconds)

    def __sub__(self, other):
        result = Duration()
        result.nanoseconds = (self.secs - other.secs) * 1000000000 + self.nsecs - other.nsecs
        return result


class Publisher(object):
    def __init__(self, topic):
        self.topic, self.messages = topic, []

    def publish(self, message):
        self.messages.append(message)


class LidarTimestampTests(unittest.TestCase):
    def setUp(self):
        self.params = {"~enable_gnss": False, "~enable_lidar": True,
                       "~lidar_output_topic": "/points_raw"}
        rospy = types.ModuleType("rospy")
        rospy.get_param = lambda name, default=None: self.params.get(name, default)
        rospy.Publisher = lambda topic, *args, **kwargs: Publisher(topic)
        rospy.Subscriber = rospy.Timer = lambda *args, **kwargs: Obj()
        rospy.Duration = Duration
        # Disable the background worker; tests advance its queue explicitly.
        rospy.on_shutdown = lambda callback: callback()
        rospy.loginfo = rospy.logwarn_throttle = lambda *args: None
        sensor = types.ModuleType("sensor_msgs")
        sensor.msg = types.ModuleType("sensor_msgs.msg")
        sensor.msg.NavSatFix = sensor.msg.NavSatStatus = sensor.msg.PointCloud2 = Obj
        std = types.ModuleType("std_msgs")
        std.msg = types.ModuleType("std_msgs.msg")
        std.msg.String = Obj
        replacements = {"rospy": rospy, "sensor_msgs": sensor, "sensor_msgs.msg": sensor.msg,
                        "std_msgs": std, "std_msgs.msg": std.msg}
        saved = {key: sys.modules.get(key) for key in replacements}
        try:
            sys.modules.update(replacements)
            module = types.ModuleType("test_sensor_fault_manager")
            filename = os.path.join(os.path.dirname(__file__), "..", "src", "fault_injections", "SensorFaultInjectionManager.py")
            with open(filename) as source:
                exec(compile(source.read(), filename, "exec"), module.__dict__)
            self.manager_type = module.SensorFaultInjectionManager
            self.node = self.manager_type()
        finally:
            for key, value in saved.items():
                if value is None:
                    sys.modules.pop(key, None)
                else:
                    sys.modules[key] = value

    def cloud(self, seq=7):
        return Obj(header=Obj(seq=seq, frame_id="os_sensor", stamp=Stamp(1700000000, 950000000)),
                   data=b"unchanged points", width=100, height=1)

    def check_offset(self, offset):
        cloud = self.cloud()
        self.node.lidar_timestamp_offset_s = offset
        self.node.lidar_callback(cloud)
        output = self.node.lidar_pub.messages[0]
        status = json.loads(self.node.lidar_timestamp_pub.messages[0].data)
        self.assertAlmostEqual(status["measured_offset_s"], offset)
        self.assertEqual(status["requested_offset_s"], offset)
        self.assertEqual(status["input_stamp"], {"secs": 1700000000, "nsecs": 950000000})
        self.assertEqual(status["output_stamp"], {"secs": output.header.stamp.secs, "nsecs": output.header.stamp.nsecs})
        self.assertEqual(status["input_topic"], "/os_cloud_node/points")
        self.assertEqual(status["output_topic"], "/points_raw")
        self.assertEqual(status["sequence"], output.header.seq)
        self.assertEqual(status["frame_id"], output.header.frame_id)
        self.assertIs(output.data, cloud.data)
        self.assertEqual(cloud.header.stamp.nsecs, 950000000)
        return status

    def test_positive_offset_crosses_second_boundary(self):
        status = self.check_offset(0.1)
        self.assertEqual(status["output_stamp"], {"secs": 1700000001, "nsecs": 50000000})

    def test_negative_offset(self):
        self.check_offset(-0.1)

    def test_normal_scan_reports_zero_offset(self):
        self.check_offset(0.0)

    def test_nanosecond_precision(self):
        self.check_offset(0.000000001)

    def test_dropped_scan_has_no_published_timestamp_pair(self):
        self.node.lidar_drop_every_n = 1
        self.node.lidar_callback(self.cloud())
        self.assertEqual(self.node.lidar_pub.messages, [])
        self.assertEqual(self.node.lidar_timestamp_pub.messages, [])

    def test_no_telemetry_publisher_when_lidar_is_disabled(self):
        self.params["~enable_lidar"] = False
        self.assertIsNone(self.manager_type().lidar_timestamp_pub)

    def command(self, command):
        self.node.command_callback(Obj(data=command))

    def test_freeze_requires_scan_and_rejects_invalid_mode(self):
        self.command("LIDAR_FREEZE_command=1")
        self.assertEqual(self.node.active_fault, "")
        self.command("LIDAR_FREEZE_command=3")
        self.assertEqual(self.node.active_fault, "")

    def test_freeze_modes_and_stop(self):
        first = self.cloud()
        self.node.lidar_callback(first)
        for mode in (1, 2):
            self.command("LIDAR_FREEZE_command=" + str(mode))
            second = self.cloud(8)
            second.data = b"new points"
            second.header.stamp = Stamp(1700000002)
            self.node.lidar_callback(second)
            output = self.node.lidar_pub.messages[-1]
            self.assertEqual(output.data, first.data)
            self.assertEqual(output.header.stamp.secs, first.header.stamp.secs if mode == 1 else second.header.stamp.secs)
            self.command("LIDAR_FREEZE_command=0")
            self.node.lidar_callback(first)
        self.assertEqual(self.node.active_fault, "")

    def test_real_delay_preserves_stamp_and_order(self):
        clock = [10.0]
        self.node.wall_clock = lambda: clock[0]
        self.command("LIDAR_DELAY_S_command=0.2")
        self.node.lidar_callback(self.cloud(1))
        clock[0] = 10.1
        self.node.lidar_callback(self.cloud(2))
        self.node._drain_delay()
        self.assertEqual(self.node.lidar_pub.messages, [])
        clock[0] = 10.25
        self.node._drain_delay()
        self.assertEqual([m.header.seq for m in self.node.lidar_pub.messages], [1])
        clock[0] = 10.4
        self.node._drain_delay()
        self.assertEqual([m.header.seq for m in self.node.lidar_pub.messages], [1, 2])
        self.assertEqual(self.node.lidar_pub.messages[0].header.stamp.nsecs, 950000000)

    def test_delay_stop_timeout_and_overflow_discard_queue(self):
        for reason in ("stop", "timeout", "overflow"):
            self.command("LIDAR_DELAY_S_command=1")
            self.node.lidar_callback(self.cloud())
            if reason == "stop":
                self.command("LIDAR_DELAY_S_command=0")
            elif reason == "timeout":
                self.node.fault_deadline = time.time() - 1
                self.node._drain_delay()
            else:
                self.node.delayed_bytes = 64 * 1024 * 1024
                self.node.lidar_callback(self.cloud())
            self.assertEqual(len(self.node.delayed_lidar), 0)
            self.assertEqual(self.node.active_fault, "")

    def test_sector_preserves_binary_fields_and_handles_padded_rows(self):
        for endian in ("<", ">"):
            front = struct.pack(endian + "fffI", 2, 0, 3, 99)
            back = struct.pack(endian + "fffI", -2, 0, 4, 123)
            message = self.cloud()
            message.fields = [Obj(name=name, offset=offset, datatype=7, count=1) for name, offset in (("x", 0), ("y", 4), ("z", 8))]
            message.width, message.height, message.point_step, message.row_step = 1, 2, 16, 20
            message.is_bigendian = endian == ">"
            message.data = front + b"pad!" + back + b"pad!"
            output = self.node._remove_sector(message, 30)
            self.assertEqual(output.data, back)
            self.assertEqual((output.width, output.height, output.row_step), (1, 1, 16))
            self.assertEqual(self.node._remove_sector(message, 360).width, 0)

    def test_single_fault_and_sector_invalid_layout(self):
        self.command("LIDAR_DELAY_S_command=0.1")
        self.command("LIDAR_SECTOR_DEG_command=30")
        self.assertEqual(self.node.active_fault, "LIDAR_DELAY_S_command")
        self.command("LIDAR_DELAY_S_command=0")
        self.command("LIDAR_SECTOR_DEG_command=30")
        message = self.cloud()
        message.fields = []
        self.node.lidar_callback(message)
        self.assertEqual(self.node.active_fault, "")
        self.assertEqual(self.node.lidar_pub.messages[-1].data, message.data)

    def point_cloud(self, points, endian="<", datatype=7):
        code = "f" if datatype == 7 else "d"
        size = struct.calcsize(code)
        record = struct.Struct(endian + code * 3 + "I")
        message = self.cloud()
        message.fields = [Obj(name=name, offset=index * size, datatype=datatype, count=1) for index, name in enumerate(("x", "y", "z"))]
        message.width, message.height = len(points), 1
        message.point_step = record.size
        message.row_step = len(points) * record.size
        message.is_bigendian = endian == ">"
        message.data = b"".join(record.pack(x, y, z, index) for index, (x, y, z) in enumerate(points))
        return message

    def test_range_uses_3d_distance_boundary_and_preserves_fields(self):
        for endian in ("<", ">"):
            for datatype in (7, 8):
                message = self.point_cloud([(3, 4, 0), (0, 0, 6), (0, 0, 0), (float("nan"), 0, 0), (float("inf"), 0, 0)], endian, datatype)
                output = self.node._limit_range(message, 5)
                step = message.point_step
                self.assertEqual(output.data, message.data[:step] + message.data[2 * step:3 * step])
                self.assertEqual((output.width, output.height), (2, 1))
                self.assertIs(output.fields, message.fields)
                self.assertEqual(message.width, 5)

    def test_range_padded_rows_and_bad_fields(self):
        message = self.point_cloud([(1, 0, 0), (10, 0, 0)])
        step = message.point_step
        message.data = message.data[:step] + b"pad!" + message.data[step:] + b"pad!"
        message.width, message.height, message.row_step = 1, 2, step + 4
        self.assertEqual(self.node._limit_range(message, 5).data, message.data[:step])
        message.fields = message.fields[:2]
        with self.assertRaises(ValueError):
            self.node._limit_range(message, 5)

    def test_point_loss_uses_percentage_and_preserves_records(self):
        message = self.point_cloud([(1, 0, 0)] * 4)
        draws = iter([0.1, 0.7, 0.2, 0.8])
        self.node.point_random = Obj(random=lambda: next(draws))
        self.command("LIDAR_POINT_LOSS_PCT_command=50")
        self.node.lidar_callback(message)
        output = self.node.lidar_pub.messages[-1]
        step = message.point_step
        self.assertEqual(output.width, 2)
        self.assertEqual(output.data, message.data[step:2 * step] + message.data[3 * step:])
        self.assertEqual(output.header.stamp.nsecs, message.header.stamp.nsecs)
        self.command("LIDAR_POINT_LOSS_PCT_command=0")
        self.command("LIDAR_POINT_LOSS_PCT_command=100")
        self.node.point_random = Obj(random=lambda: 0.99)
        self.node.lidar_callback(message)
        self.assertEqual(self.node.lidar_pub.messages[-1].data, b"")

    def test_new_filters_stop_timeout_and_single_fault(self):
        for fault, value in [("LIDAR_RANGE_M_command", 5), ("LIDAR_POINT_LOSS_PCT_command", 50)]:
            self.command(fault + "=" + str(value))
            self.command("LIDAR_SECTOR_DEG_command=30")
            self.assertEqual(self.node.active_fault, fault)
            self.node.fault_deadline = time.time() - 1
            message = self.point_cloud([(10, 0, 0)])
            self.node.lidar_callback(message)
            self.assertEqual(self.node.active_fault, "")
            self.assertEqual(self.node.lidar_pub.messages[-1].data, message.data)
            self.command(fault + "=" + str(value))
            self.command(fault + "=0")
            self.node.lidar_callback(message)
            self.assertEqual(self.node.lidar_pub.messages[-1].data, message.data)

    def test_new_filter_commands_reject_invalid_values_and_report_limits(self):
        for command in ["LIDAR_RANGE_M_command=-1", "LIDAR_RANGE_M_command=nan", "LIDAR_POINT_LOSS_PCT_command=101", "LIDAR_POINT_LOSS_PCT_command=-1"]:
            # Invalid numeric input uses the real command parser's warning path.
            self.node.command_callback.__globals__["rospy"].logwarn = lambda *args: None
            self.command(command)
            self.assertEqual(self.node.active_fault, "")
        self.node.config_callback(Obj(data="FAULT_LIMITS_QUERY_command"))
        faults = [json.loads(message.data)["fault"] for message in self.node.status_pub.messages]
        self.assertIn("LIDAR_RANGE_M_command", faults)
        self.assertIn("LIDAR_POINT_LOSS_PCT_command", faults)

    def test_translations_support_signed_values_all_axes_and_keep_binary_fields(self):
        for endian in ("<", ">"):
            for datatype in (7, 8):
                for axis in range(3):
                    message = self.point_cloud([(1, 2, 3)], endian, datatype)
                    output = self.node._transform_cloud(message, "translate", axis, -0.5)
                    reader = struct.Struct(endian + ("f" if datatype == 7 else "d") * 3)
                    expected = [1, 2, 3]
                    expected[axis] -= 0.5
                    self.assertEqual(list(reader.unpack_from(output.data)), expected)
                    self.assertEqual(output.data[reader.size:], message.data[reader.size:])
                    self.assertEqual(message.data, self.point_cloud([(1, 2, 3)], endian, datatype).data)
                    self.assertIs(output.header, message.header)

    def test_rotation_right_hand_rule_all_axes_and_signed_angles(self):
        for axis, point, expected in [(0, (0, 1, 0), (0, 0, 1)),
                                      (1, (0, 0, 1), (1, 0, 0)),
                                      (2, (1, 0, 0), (0, 1, 0))]:
            for angle, sign in [(90, 1), (-90, -1), (450, 1)]:
                message = self.point_cloud([point])
                output = self.node._transform_cloud(message, "rotate", axis, angle)
                coordinates = struct.unpack_from("<fff", output.data)
                for actual, target in zip(coordinates, expected):
                    self.assertAlmostEqual(actual, sign * target, places=6)

    def test_transform_keeps_organised_layout_padding_and_invalid_points(self):
        message = self.point_cloud([(1, 2, 3), (float("nan"), 4, 5)])
        step = message.point_step
        message.data = message.data[:step] + b"pad!" + message.data[step:] + b"end!"
        original = message.data
        message.width, message.height, message.row_step = 1, 2, step + 4
        output = self.node._transform_cloud(message, "translate", 0, 2)
        self.assertEqual((output.width, output.height, output.row_step), (1, 2, step + 4))
        self.assertEqual(output.data[step:], original[step:])
        self.assertEqual(struct.unpack_from("<f", output.data)[0], 3)
        self.assertEqual(message.data, original)

    def test_transform_commands_stop_timeout_limits_and_single_fault(self):
        commands = ["LIDAR_TRANSLATE_" + axis + "_M_command" for axis in ("X", "Y", "Z")]
        commands += ["LIDAR_" + axis + "_DEG_command" for axis in ("ROLL", "PITCH", "YAW")]
        for command in commands:
            self.command(command + "=-2")
            self.assertEqual(self.node.active_fault, command)
            self.assertEqual(self.node._active_fault_value(), -2)
            self.command("LIDAR_RANGE_M_command=10")
            self.assertEqual(self.node.active_fault, command)
            self.node.lidar_callback(self.point_cloud([(1, 2, 3)]))
            self.command(command + "=0")
            self.assertEqual(self.node.lidar_transform_fault, "")
            self.command(command + "=-2")
            self.node.fault_deadline = time.time() - 1
            message = self.point_cloud([(1, 2, 3)])
            self.node.lidar_callback(message)
            self.assertEqual(self.node.lidar_pub.messages[-1].data, message.data)
            self.assertEqual(self.node.lidar_transform_fault, "")
        self.node.config_callback(Obj(data="FAULT_LIMITS_QUERY_command"))
        reports = [json.loads(message.data) for message in self.node.status_pub.messages]
        for command in commands:
            self.assertTrue(any(report["fault"] == command and report["event"] == "limits" for report in reports))

    def test_bad_transform_layout_or_overflow_stops_fault_and_passes_original(self):
        for kind in ("missing", "overlap", "overflow"):
            message = self.point_cloud([(1, 2, 3)])
            value = "1"
            if kind == "missing":
                message.fields.pop()
            elif kind == "overlap":
                message.fields[1].offset = 0
            else:
                value = "1e100"  # finite input, but cannot fit FLOAT32 output
            self.command("LIDAR_TRANSLATE_X_M_command=" + value)
            self.node.lidar_callback(message)
            self.assertEqual(self.node.active_fault, "")
            self.assertEqual(self.node.lidar_pub.messages[-1].data, message.data)

    def test_noise_changes_xyz_only_and_preserves_invalid_points_and_layout(self):
        for endian in ("<", ">"):
            for datatype in (7, 8):
                message = self.point_cloud([(1, 2, 3), (float("nan"), 4, 5)], endian, datatype)
                step = message.point_step
                message.data = message.data[:step] + b"pad!" + message.data[step:] + b"end!"
                message.width, message.height, message.row_step = 1, 2, step + 4
                draws = iter([0.1, -0.2, 0.3])
                calls = []
                def gauss(mean, sigma):
                    calls.append((mean, sigma))
                    return next(draws)
                output = self.node._transform_cloud(message, "noise", 0, 0.05, Obj(gauss=gauss))
                reader = struct.Struct(endian + ("f" if datatype == 7 else "d") * 3)
                for actual, expected in zip(reader.unpack_from(output.data), (1.1, 1.8, 3.3)):
                    self.assertAlmostEqual(actual, expected, places=6)
                self.assertEqual(output.data[reader.size:], message.data[reader.size:])
                self.assertEqual((output.width, output.height, output.row_step), (1, 2, step + 4))
                self.assertIs(output.header, message.header)
                self.assertEqual(calls, [(0.0, 0.05)] * 3)

    def test_noise_distribution_matches_selected_standard_deviation(self):
        message = self.point_cloud([(0, 0, 0)] * 2000)
        output = self.node._transform_cloud(message, "noise", 0, 0.05, random.Random(17))
        values = [struct.unpack_from("<f", output.data, start)[0]
                  for start in range(0, len(output.data), message.point_step)]
        mean = sum(values) / len(values)
        spread = (sum((value - mean) ** 2 for value in values) / len(values)) ** 0.5
        self.assertLess(abs(mean), 0.005)
        self.assertAlmostEqual(spread, 0.05, delta=0.005)

    def test_keep_interval_forwards_nth_scans_and_restarts_counter(self):
        for repeat in range(2):
            self.command("LIDAR_KEEP_EVERY_N_command=3")
            before = len(self.node.lidar_pub.messages)
            for seq in range(1, 8):
                self.node.lidar_callback(self.cloud(seq))
            self.assertEqual([message.header.seq for message in self.node.lidar_pub.messages[before:]], [3, 6])
            self.assertEqual(len(self.node.lidar_timestamp_pub.messages), (repeat + 1) * 2)
            self.command("LIDAR_KEEP_EVERY_N_command=0")
        self.node.lidar_callback(self.cloud(8))
        self.assertEqual(self.node.lidar_pub.messages[-1].header.seq, 8)

    def test_noise_and_keep_timeout_stop_recovery_and_conflict(self):
        for fault, value in [("LIDAR_NOISE_STDDEV_M_command", "0.05"), ("LIDAR_KEEP_EVERY_N_command", "2")]:
            self.command(fault + "=" + value)
            self.command("LIDAR_RANGE_M_command=10")
            self.assertEqual(self.node.active_fault, fault)
            self.node.fault_deadline = time.time() - 1
            message = self.point_cloud([(1, 2, 3)])
            self.node.lidar_callback(message)
            self.assertEqual(self.node.lidar_pub.messages[-1].data, message.data)
            self.command(fault + "=" + value)
            self.command(fault + "=0")
            self.assertEqual(self.node.active_fault, "")
            self.command(fault + "=" + value)
            # Recovery calls this same cleanup routine under the lock.
            self.node._clear_sensor_faults()
            self.assertEqual(self.node.lidar_keep_every_n, 0)
            self.assertEqual(self.node.lidar_keep_count, 0)
            self.assertEqual(self.node.lidar_noise_stddev_m, 0)

    def test_noise_overflow_stops_fault_and_forwards_original(self):
        self.command("LIDAR_NOISE_STDDEV_M_command=0.05")
        self.node.point_random = Obj(gauss=lambda *args: 1e100)
        message = self.point_cloud([(1, 2, 3)])
        self.node.lidar_callback(message)
        self.assertEqual(self.node.active_fault, "")
        self.assertEqual(self.node.lidar_pub.messages[-1].data, message.data)

    def test_noise_and_keep_invalid_values_limits_and_integer_config(self):
        self.node.command_callback.__globals__["rospy"].logwarn = lambda *args: None
        for command in ["LIDAR_NOISE_STDDEV_M_command=-1", "LIDAR_NOISE_STDDEV_M_command=nan",
                        "LIDAR_KEEP_EVERY_N_command=1", "LIDAR_KEEP_EVERY_N_command=2.5",
                        "LIDAR_KEEP_EVERY_N_command=9007199254740992"]:
            self.command(command)
            self.assertEqual(self.node.active_fault, "")
        self.node.config_callback(Obj(data="FAULT_LIMITS_QUERY_command"))
        reports = [json.loads(message.data) for message in self.node.status_pub.messages]
        for command in ("LIDAR_NOISE_STDDEV_M_command", "LIDAR_KEEP_EVERY_N_command"):
            self.assertTrue(any(report["fault"] == command and report["event"] == "limits" for report in reports))
        self.node.config_callback(Obj(data="FAULT_LIMITS_command=LIDAR_KEEP_EVERY_N_command,2.5,5"))
        self.assertEqual(self.node.fault_ranges["LIDAR_KEEP_EVERY_N_command"], [2, 20])


if __name__ == "__main__":
    unittest.main()
