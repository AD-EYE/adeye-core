#!/usr/bin/env python

"""Run one bounded steering-saturation fault experiment from a terminal."""

import rospy
from autoware_msgs.msg import VehicleStatus
from std_msgs.msg import String


DEFAULT_STEERING_LIMIT = 0.10
DEFAULT_MAXIMUM_SPEED = 0.50
DEFAULT_DURATION = 2.0
MAXIMUM_ALLOWED_SPEED = 1.00
STATUS_TIMEOUT = 10.0


try:
    prompt = raw_input
except NameError:
    prompt = input


class SteeringSaturationExperiment:

    def __init__(self):
        self.current_speed = None
        self.command_pub = rospy.Publisher(
            "/vehicle_commands", String, queue_size=1
        )
        rospy.Subscriber(
            "/vehicle_status", VehicleStatus, self.vehicle_status_callback
        )

    def vehicle_status_callback(self, message):
        self.current_speed = message.speed

    def publish_command(self, command):
        rospy.loginfo("Fault command: %s", command)
        self.command_pub.publish(String(data=command))

    def reset_fault(self):
        self.publish_command("STEERSAT_command=0")

    def wait_for_vehicle_status(self):
        deadline = rospy.Time.now() + rospy.Duration(STATUS_TIMEOUT)
        while not rospy.is_shutdown() and self.current_speed is None:
            if rospy.Time.now() >= deadline:
                rospy.logerr("No /vehicle_status received; experiment aborted.")
                return False
            rospy.sleep(0.1)
        return not rospy.is_shutdown()

    def run(self, steering_limit, maximum_speed, duration):
        if maximum_speed <= 0.0 or maximum_speed > MAXIMUM_ALLOWED_SPEED:
            rospy.logerr(
                "Maximum speed must be greater than 0 and no more than %.2f m/s.",
                MAXIMUM_ALLOWED_SPEED,
            )
            return

        if steering_limit <= 0.0 or duration <= 0.0:
            rospy.logerr("Steering limit and duration must both be positive.")
            return

        if not self.wait_for_vehicle_status():
            return

        if abs(self.current_speed) > maximum_speed:
            rospy.logerr(
                "Vehicle speed %.2f m/s exceeds the %.2f m/s limit; aborting.",
                self.current_speed,
                maximum_speed,
            )
            return

        self.reset_fault()

        try:
            rospy.logwarn(
                "Applying steering saturation %.3f for %.1f seconds.",
                steering_limit,
                duration,
            )
            self.publish_command(
                "STEERSAT_command={}".format(steering_limit)
            )

            end_time = rospy.Time.now() + rospy.Duration(duration)
            while not rospy.is_shutdown() and rospy.Time.now() < end_time:
                if abs(self.current_speed) > maximum_speed:
                    rospy.logerr("Speed limit exceeded; resetting the fault.")
                    return
                rospy.sleep(0.05)
        finally:
            rospy.logwarn("Resetting steering saturation.")
            self.reset_fault()


def read_float(label, default):
    value = prompt("{} [{}]: ".format(label, default)).strip()
    return default if not value else float(value)


def main():
    rospy.init_node("steering_saturation_experiment")
    experiment = SteeringSaturationExperiment()

    print("")
    print("===============================================")
    print(" AD-EYE Bounded Steering Saturation Experiment")
    print("===============================================")
    print("Use only in simulation or a closed test area with")
    print("an independent emergency-stop path and safety driver.")
    print("The fault always resets when this experiment ends.")
    print("")

    try:
        steering_limit = read_float("Steering limit", DEFAULT_STEERING_LIMIT)
        maximum_speed = read_float("Maximum speed (m/s)", DEFAULT_MAXIMUM_SPEED)
        duration = read_float("Fault duration (s)", DEFAULT_DURATION)
    except ValueError:
        rospy.logerr("All values must be valid numbers.")
        return

    if prompt("Type ARM to start: ").strip() != "ARM":
        rospy.logwarn("Experiment was not armed.")
        return

    experiment.run(steering_limit, maximum_speed, duration)


if __name__ == "__main__":
    main()
