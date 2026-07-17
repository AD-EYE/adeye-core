#!/usr/bin/env python

import rospy
from std_msgs.msg import String

# rosrun adeye fault_test.py


class FaultTester:

    def __init__(self):

        self.pub = rospy.Publisher("/vehicle_commands", String, queue_size=1)

        self.state_pub = rospy.Publisher("/state_cmd", String, queue_size=1)

    def run_test(self, name, command, duration=5):

        rospy.loginfo("==============================")
        rospy.loginfo("START TEST: " + name)
        rospy.loginfo("COMMAND: " + command)

        self.pub.publish(command)

        rospy.sleep(duration)

        rospy.loginfo("END TEST: " + name)
        rospy.loginfo("==============================")

    def test_steering_offset(self):

        self.run_test("STEERING OFFSET +10", "STEEROFFSET_command=10", 5)

        self.run_test("STEERING OFFSET RESET", "STEEROFFSET_command=0", 2)

    def test_steering_freeze(self):

        self.run_test("STEERING FREEZE ON", "STEERFREEZE_command=1", 5)

        self.run_test("STEERING FREEZE OFF", "STEERFREEZE_command=0", 2)

    def test_steering_saturation(self):

        self.run_test("STEERING SATURATION +/-10", "STEERSAT_command=10", 5)

        self.run_test("STEERING SATURATION RESET", "STEERSAT_command=0", 2)

    def test_steering_oscillation(self):

        self.run_test("STEERING OSCILLATION AMP=5", "STEEROSC_command=5", 10)

        self.run_test("STEERING OSCILLATION RESET", "STEEROSC_command=0", 2)

    def test_steering_random(self):

        self.run_test("STEERING RANDOM ON", "STEERRANDOM_command=1", 10)

        self.run_test("STEERING RANDOM OFF", "STEERRANDOM_command=0", 2)

    def test_acceleration_offset(self):

        self.run_test("ACCELERATION OFFSET +2", "ACCELOFFSET_command=2", 5)

        self.run_test("ACCELERATION OFFSET RESET", "ACCELOFFSET_command=0", 2)

    def test_acceleration_freeze(self):

        self.run_test("ACCELERATION FREEZE ON", "ACCELFREEZE_command=1", 5)

        self.run_test("ACCELERATION FREEZE OFF", "ACCELFREEZE_command=0", 2)

    def test_acceleration_saturation(self):

        self.run_test("ACCELERATION SATURATION 1", "ACCELSAT_command=1", 5)

        self.run_test("ACCELERATION SATURATION RESET", "ACCELSAT_command=0", 2)

    def test_acceleration_oscillation(self):

        self.run_test("ACCELERATION OSCILLATION", "ACCELOSC_command=2", 10)

        self.run_test("ACCELERATION OSCILLATION RESET", "ACCELOSC_command=0", 2)

    def test_acceleration_runaway(self):

        self.run_test("RUNAWAY ACCELERATION", "ACCELRUNAWAY_command=1", 5)

        self.run_test("RUNAWAY ACCELERATION OFF", "ACCELRUNAWAY_command=0", 2)

    def reset_all_faults(self):

        rospy.loginfo("RESETTING ALL FAULTS")

        self.state_pub.publish("return_to_ready")

        self.pub.publish("STEEROFFSET_command=0")
        self.pub.publish("STEERFREEZE_command=0")
        self.pub.publish("STEERSAT_command=0")
        self.pub.publish("STEEROSC_command=0")
        self.pub.publish("STEERRANDOM_command=0")

        self.pub.publish("ACCELOFFSET_command=0")
        self.pub.publish("ACCELFREEZE_command=0")
        self.pub.publish("ACCELSAT_command=0")
        self.pub.publish("ACCELOSC_command=0")
        self.pub.publish("ACCELRUNAWAY_command=0")

        self.pub.publish("WLOCK_command=1")
        self.pub.publish("HL_command=0")

        rospy.sleep(2)

    def test_emergency_state(self):

        rospy.loginfo("STARTING EMERGENCY TEST")

        self.state_pub.publish("emergency")

        rospy.sleep(5)

        self.state_pub.publish("return_to_ready")

        rospy.sleep(2)

        rospy.loginfo("EMERGENCY TEST FINISHED")


if __name__ == "__main__":

    rospy.init_node("fault_tester")

    tester = FaultTester()

    tester.reset_all_faults()

    tester.run_test("HAZARD ON", "HL_command=1", 5)

    tester.run_test("HAZARD OFF", "HL_command=0", 2)

    tester.run_test("STEERING LOCK ON", "WLOCK_command=0", 5)

    tester.run_test("STEERING LOCK OFF", "WLOCK_command=1", 2)

    tester.test_steering_offset()
    rospy.sleep(2)

    tester.test_steering_freeze()
    rospy.sleep(2)

    tester.test_steering_saturation()
    rospy.sleep(2)

    tester.test_steering_oscillation()
    rospy.sleep(2)

    tester.test_steering_random()

    tester.test_acceleration_offset()
    rospy.sleep(2)

    tester.test_acceleration_freeze()
    rospy.sleep(2)

    tester.test_acceleration_saturation()
    rospy.sleep(2)

    tester.test_acceleration_oscillation()
    rospy.sleep(2)

    tester.test_acceleration_runaway()
    rospy.sleep(2)

    tester.test_emergency_state()

    tester.reset_all_faults()

    rospy.loginfo("ALL TESTS FINISHED")
