"""Exercise real file logging with a small ROS stand-in."""
import glob
import gzip
import importlib.util
import json
import os
import sys
import tempfile
import threading
import types
import unittest
from unittest import mock


class Message(object):
    def __init__(self, data=""):
        self.data = data


class FaultLoggerTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.warnings, self.published = [], []
        self.ros = types.ModuleType("rospy")
        self.ros.ROSException = type("ROSException", (Exception,), {})
        self.ros.is_shutdown = lambda: False
        self.ros.get_param = lambda name, default=None: self.directory.name if name == "~fault_log_dir" else default
        self.ros.loginfo = lambda *args: None
        self.ros.logwarn = lambda *args: self.warnings.append(args)
        self.ros.logwarn_throttle = lambda period, *args: self.warnings.append(args)
        self.ros.on_shutdown = lambda callback: None
        self.ros.Subscriber = lambda *args, **kwargs: None
        def publisher(topic, *args, **kwargs):
            return types.SimpleNamespace(publish=lambda message: self.published.append((topic, message.data)))
        self.ros.Publisher = publisher
        standard = types.ModuleType("std_msgs.msg")
        standard.String = Message
        autoware = types.ModuleType("autoware_msgs.msg")
        autoware.VehicleCmd = Message
        path = os.path.join(os.path.dirname(__file__), "..", "src", "fault_injections", "FaultInjectionManager.py")
        spec = importlib.util.spec_from_file_location("fault_logger_under_test", path)
        self.module = importlib.util.module_from_spec(spec)
        with mock.patch.dict(sys.modules, {"rospy": self.ros, "std_msgs.msg": standard,
                                           "autoware_msgs.msg": autoware}):
            spec.loader.exec_module(self.module)
        self.node = self.module.FaultInjectionManager(None)
        self.addCleanup(self.node._close_fault_log)

    def status(self):
        return Message(json.dumps({"source": "steering", "event": "limits",
                                   "fault": "STEEROFFSET_command", "active": False}))

    def test_valid_status_is_written_and_published_to_both_topics(self):
        self.node.physicalFaultStatusCallback(self.status())
        with open(self.node.fault_log_path) as source:
            self.assertIn("PHYSICAL_FAULT_STATUS:steering:STEEROFFSET_command", source.read())
        self.assertEqual([topic for topic, _ in self.published], ["/adeye/fault_log", "/fault_log"])
        self.assertEqual(self.warnings, [])

    def test_invalid_json_and_non_object_are_reported_without_crashing(self):
        for value in ["not json", "[]", "null", "123"]:
            self.node.physicalFaultStatusCallback(Message(value))
        self.assertEqual(len(self.warnings), 4)
        self.assertTrue(all("Invalid physical fault status JSON" in args[0] for args in self.warnings))

    def test_unexpected_closed_file_is_reopened(self):
        self.node.fault_log.close()
        self.node.physicalFaultStatusCallback(self.status())
        self.assertFalse(self.node.fault_log.closed)
        self.assertEqual(self.warnings, [])
        self.assertEqual(len(self.published), 2)

    def test_unwritable_file_does_not_get_called_invalid_json(self):
        self.node.fault_log.close()
        with mock.patch.object(self.node, "_open_new_fault_log", side_effect=IOError("disk unavailable")):
            self.node.physicalFaultStatusCallback(self.status())
        self.assertEqual(len(self.published), 2)
        self.assertEqual(len(self.warnings), 1)
        self.assertIn("Fault log file could not be written", self.warnings[0][0])

    def test_failed_rotation_keeps_previous_file_usable(self):
        self.node.fault_log_max_bytes = 1
        previous = self.node.fault_log
        with mock.patch.object(self.node, "_open_new_fault_log", side_effect=IOError("cannot create next file")):
            self.node.logFault("TEST", "still saved", force=True)
        self.assertIs(self.node.fault_log, previous)
        self.assertFalse(previous.closed)
        with open(self.node.fault_log_path) as source:
            self.assertIn("still saved", source.read())
        self.assertEqual(len(self.published), 2)

    def test_disk_write_failure_still_publishes_status(self):
        original = self.node.fault_log
        self.node.fault_log = mock.Mock(closed=False)
        self.node.fault_log.write.side_effect = IOError("disk full")
        try:
            self.node.physicalFaultStatusCallback(self.status())
            self.assertEqual(len(self.published), 2)
            self.assertIn("Fault log file could not be written", self.warnings[0][0])
            self.assertNotIn("Invalid physical fault status JSON", str(self.warnings))
        finally:
            self.node.fault_log = original

    def test_compressed_rotation_preserves_records(self):
        self.node.fault_log_max_bytes = 1
        self.node.fault_log_keep_files = 20
        for number in range(5):
            self.node.logFault("COMPRESSED_%d" % number, "value", force=True)
        compressed = glob.glob(os.path.join(self.directory.name, "*.gz"))
        self.assertEqual(len(compressed), 5)
        content = b""
        for path in compressed:
            with gzip.open(path, "rb") as source:
                content += source.read()
        for number in range(5):
            self.assertIn(("COMPRESSED_%d" % number).encode("ascii"), content)

    def test_parallel_callbacks_and_rotation_keep_every_record(self):
        self.node.fault_log_max_bytes = 200
        self.node.fault_log_compress_rotated = False
        self.node.fault_log_keep_files = 200
        errors = []
        def write(worker):
            try:
                for number in range(20):
                    self.node.logFault("TEST_%d_%d" % (worker, number), "value", force=True)
            except Exception as error:
                errors.append(error)
        threads = [threading.Thread(target=write, args=(worker,)) for worker in range(4)]
        for thread in threads:
            thread.start()
        for thread in threads:
            thread.join(5)
            self.assertFalse(thread.is_alive())
        self.assertEqual(errors, [])
        records = []
        for path in glob.glob(os.path.join(self.directory.name, "*.txt")):
            with open(path) as source:
                records.extend(line for line in source if " | TEST_" in line)
        self.assertEqual(len(records), 80)
        self.assertEqual(len(self.published), 160)
        self.assertEqual(self.warnings, [])

    def test_shutdown_waits_for_current_write_and_ignores_late_callbacks(self):
        entered, release, closed = threading.Event(), threading.Event(), threading.Event()
        errors = []
        def finish_write():
            entered.set()
            if not release.wait(3):
                raise RuntimeError("test writer timed out")
        def write():
            try:
                self.node.logFault("TEST", "before close", force=True)
            except Exception as error:
                errors.append(error)
        def close():
            self.node._close_fault_log()
            closed.set()
        with mock.patch.object(self.node, "_rotate_fault_log_if_needed", side_effect=finish_write):
            writer = threading.Thread(target=write)
            writer.start()
            self.assertTrue(entered.wait(3))
            closer = threading.Thread(target=close)
            closer.start()
            try:
                self.assertFalse(closed.wait(0.03))
            finally:
                release.set()
                writer.join(5)
                closer.join(5)
        self.assertEqual(errors, [])
        self.assertTrue(closed.is_set())
        count = len(self.published)
        self.node.physicalFaultStatusCallback(self.status())
        self.node.logFaultDebug("LATE", "ignored")
        self.node._close_fault_log()
        self.assertTrue(self.node.fault_log.closed)
        self.assertEqual(len(self.published), count)

    def test_one_closed_topic_does_not_block_the_other(self):
        self.node.fault_debug_pub.publish = mock.Mock(side_effect=self.ros.ROSException("closed topic"))
        self.node.physicalFaultStatusCallback(self.status())
        self.assertEqual([topic for topic, _ in self.published], ["/fault_log"])
        self.assertNotIn("Invalid physical fault status JSON", str(self.warnings))

    def test_unwritable_startup_does_not_prevent_ros_logging(self):
        with mock.patch.object(self.module, "open", side_effect=IOError("permission denied"), create=True):
            node = self.module.FaultInjectionManager(None)
            try:
                node.physicalFaultStatusCallback(self.status())
                self.assertIsNone(node.fault_log)
                self.assertEqual(len(self.published), 2)
            finally:
                node._close_fault_log()


if __name__ == "__main__":
    unittest.main()
