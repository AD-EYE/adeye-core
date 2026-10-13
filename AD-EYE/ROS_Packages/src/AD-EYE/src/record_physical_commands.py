#!/usr/bin/env python

"""Record final physical steering and acceleration requests in a dated bag."""

from __future__ import print_function

import datetime
import os
import signal
import subprocess

import rospy

try:
    string_types = (basestring,)
except NameError:
    string_types = (str,)


class PhysicalCommandRecorder:
    def __init__(self):
        output_directory = os.path.abspath(
            os.path.expanduser(
                rospy.get_param(
                    "~output_directory", "/media/Ubuntu/rosbag_recording"
                )
            )
        )
        bag_prefix = rospy.get_param("~bag_prefix", "physical_commands")
        topics = rospy.get_param(
            "~topics",
            ["/acceleration_requested_phy", "/steering_requested_phy"],
        )

        if not topics or not all(isinstance(topic, string_types) for topic in topics):
            raise rospy.ROSInitException("~topics must be a non-empty string list")

        if not os.path.isdir(output_directory):
            try:
                os.makedirs(output_directory)
            except OSError as error:
                if not os.path.isdir(output_directory):
                    raise rospy.ROSInitException(
                        "Cannot create bag directory %s: %s"
                        % (output_directory, error)
                    )

        if not os.access(output_directory, os.W_OK):
            raise rospy.ROSInitException(
                "Bag directory is not writable: %s" % output_directory
            )

        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        bag_path = os.path.join(
            output_directory, "%s_%s.bag" % (bag_prefix, timestamp)
        )
        command = ["rosbag", "record", "-O", bag_path] + topics

        rospy.loginfo("Saving car command recording to %s", bag_path)
        rospy.loginfo("Recording topics: %s", ", ".join(topics))
        try:
            self.process = subprocess.Popen(command, preexec_fn=os.setsid)
        except OSError as error:
            raise rospy.ROSInitException("Could not start rosbag record: %s" % error)

        rospy.on_shutdown(self.stop)

    def stop(self):
        if self.process.poll() is None:
            rospy.loginfo("Stopping car command recording")
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGINT)
                self.process.wait()
            except OSError as error:
                rospy.logwarn("Could not stop the rosbag recorder normally: %s", error)

    def monitor(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            return_code = self.process.poll()
            if return_code is not None:
                rospy.logerr("Rosbag recording stopped unexpectedly with code %s", return_code)
                return
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("physical_command_bag_recorder")
    recorder = PhysicalCommandRecorder()
    recorder.monitor()
