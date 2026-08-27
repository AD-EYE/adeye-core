#!/usr/bin/env python

import rospy
from std_msgs.msg import String


try:
    read_input = raw_input
except NameError:
    read_input = input


class FaultTester:

    def __init__(self):

        self.pub = rospy.Publisher("/vehicle_commands", String, queue_size=1)
        self.state_pub = rospy.Publisher("/state_cmd", String, queue_size=1)

    def run_fault(self, name, command_on, command_off=None, duration=5):

        rospy.loginfo("====================================")
        rospy.loginfo("Running: " + name)
        rospy.loginfo(command_on)

        self.pub.publish(command_on)

        try:
            rospy.sleep(duration)
        finally:
            if command_off is not None:
                self.pub.publish(command_off)
                rospy.sleep(2)

        rospy.loginfo("Finished: " + name)
        rospy.loginfo("====================================")

    def emergency_test(self):

        rospy.loginfo("Emergency ON")
        self.state_pub.publish("emergency")

        try:
            rospy.sleep(5)
        finally:
            rospy.loginfo("Emergency OFF")
            self.state_pub.publish("return_to_ready")
            rospy.sleep(2)

    def reset(self):

        rospy.loginfo("Resetting all faults...")

        self.state_pub.publish("return_to_ready")

        commands = [
            "STEEROFFSET_command=0",
            "STEERFREEZE_command=0",
            "STEERSAT_command=0",
            "STEEROSC_command=0",
            "STEERRANDOM_command=0",
            "ACCELOFFSET_command=0",
            "ACCELFREEZE_command=0",
            "ACCELSAT_command=0",
            "ACCELOSC_command=0",
            "ACCELRUNAWAY_command=0",
            "WLOCK_command=1",
            "HL_command=0",
            "GNSS_EAST_BIAS_M_command=0",
            "GNSS_NORTH_BIAS_M_command=0",
            "GNSS_DROPOUT_command=0",
            "GNSS_NO_FIX_command=0",
            "LIDAR_DROP_EVERY_N_command=0",
            "LIDAR_TIMESTAMP_OFFSET_S_command=0",
        ]

        for cmd in commands:
            self.pub.publish(cmd)

        rospy.sleep(2)

    def localization_input_loss_test(self, duration=5):

        rospy.logwarn("Starting GNSS and LiDAR input-loss test")
        self.pub.publish("GNSS_DROPOUT_command=1")
        self.pub.publish("LIDAR_DROP_EVERY_N_command=1")

        try:
            rospy.sleep(duration)
        finally:
            self.pub.publish("GNSS_DROPOUT_command=0")
            self.pub.publish("LIDAR_DROP_EVERY_N_command=0")
            rospy.logwarn("GNSS and LiDAR input-loss test reset")


def ask_float(text, default):

    value = read_input(text)

    if value == "":
        return default

    return float(value)


if __name__ == "__main__":

    rospy.init_node("fault_tester")

    tester = FaultTester()

    tester.reset()

    while not rospy.is_shutdown():

        print("")
        print("==========================================")
        print("      AD-EYE Fault Injection Menu")
        print("==========================================")
        print(" 1  Hazard lights")
        print(" 2  Steering lock")
        print(" 3  Steering offset")
        print(" 4  Steering freeze")
        print(" 5  Steering saturation")
        print(" 6  Steering oscillation")
        print(" 7  Steering random (simulation only)")
        print("")
        print(" 8  Acceleration offset")
        print(" 9  Acceleration freeze")
        print("10  Acceleration saturation")
        print("11  Acceleration oscillation")
        print("12  Runaway acceleration (simulation only)")
        print("")
        print("13  Emergency state")
        print("14  Reset all faults")
        print("15  Run all tests (simulation only)")
        print("")
        print("16  GNSS east bias")
        print("17  GNSS north bias")
        print("18  GNSS dropout")
        print("19  GNSS no-fix status")
        print("20  LiDAR periodic scan dropout")
        print("21  LiDAR timestamp offset")
        print("22  Localization input loss (GNSS + LiDAR)")
        print(" 0  Exit")
        print("==========================================")

        try:
            choice = int(read_input("Selection: "))
        except ValueError:
            print("Selection must be an integer from 0 to 22.")
            continue

        if choice == 0:
            break

        elif choice == 1:

            tester.run_fault(
                "Hazard Lights",
                "HL_command=1",
                "HL_command=0",
            )

        elif choice == 2:

            tester.run_fault(
                "Steering Lock",
                "WLOCK_command=0",
                "WLOCK_command=1",
            )

        elif choice == 3:

            value = ask_float("Offset (default 10): ", 10)

            tester.run_fault(
                "Steering Offset",
                "STEEROFFSET_command={}".format(value),
                "STEEROFFSET_command=0",
            )

        elif choice == 4:

            tester.run_fault(
                "Steering Freeze",
                "STEERFREEZE_command=1",
                "STEERFREEZE_command=0",
            )

        elif choice == 5:

            value = ask_float("Limit (default 10): ", 10)

            tester.run_fault(
                "Steering Saturation",
                "STEERSAT_command={}".format(value),
                "STEERSAT_command=0",
            )

        elif choice == 6:

            value = ask_float("Amplitude (default 5): ", 5)

            tester.run_fault(
                "Steering Oscillation",
                "STEEROSC_command={}".format(value),
                "STEEROSC_command=0",
                10,
            )

        elif choice == 7:

            tester.run_fault(
                "Steering Random",
                "STEERRANDOM_command=1",
                "STEERRANDOM_command=0",
                10,
            )

        elif choice == 8:

            value = ask_float("Offset (default 2): ", 2)

            tester.run_fault(
                "Acceleration Offset",
                "ACCELOFFSET_command={}".format(value),
                "ACCELOFFSET_command=0",
            )

        elif choice == 9:

            tester.run_fault(
                "Acceleration Freeze",
                "ACCELFREEZE_command=1",
                "ACCELFREEZE_command=0",
            )

        elif choice == 10:

            value = ask_float("Limit (default 1): ", 1)

            tester.run_fault(
                "Acceleration Saturation",
                "ACCELSAT_command={}".format(value),
                "ACCELSAT_command=0",
            )

        elif choice == 11:

            value = ask_float("Amplitude (default 2): ", 2)

            tester.run_fault(
                "Acceleration Oscillation",
                "ACCELOSC_command={}".format(value),
                "ACCELOSC_command=0",
                10,
            )

        elif choice == 12:

            tester.run_fault(
                "Runaway Acceleration",
                "ACCELRUNAWAY_command=1",
                "ACCELRUNAWAY_command=0",
            )

        elif choice == 13:

            tester.emergency_test()

        elif choice == 14:

            tester.reset()

        elif choice == 15:

            tests = [
                ("Hazard", "HL_command=1", "HL_command=0", 5),
                ("Wheel Lock", "WLOCK_command=0", "WLOCK_command=1", 5),
                (
                    "Steering Offset",
                    "STEEROFFSET_command=10",
                    "STEEROFFSET_command=0",
                    5,
                ),
                (
                    "Steering Freeze",
                    "STEERFREEZE_command=1",
                    "STEERFREEZE_command=0",
                    5,
                ),
                ("Steering Saturation", "STEERSAT_command=10", "STEERSAT_command=0", 5),
                (
                    "Steering Oscillation",
                    "STEEROSC_command=5",
                    "STEEROSC_command=0",
                    10,
                ),
                (
                    "Steering Random",
                    "STEERRANDOM_command=1",
                    "STEERRANDOM_command=0",
                    10,
                ),
                (
                    "Acceleration Offset",
                    "ACCELOFFSET_command=2",
                    "ACCELOFFSET_command=0",
                    5,
                ),
                (
                    "Acceleration Freeze",
                    "ACCELFREEZE_command=1",
                    "ACCELFREEZE_command=0",
                    5,
                ),
                (
                    "Acceleration Saturation",
                    "ACCELSAT_command=1",
                    "ACCELSAT_command=0",
                    5,
                ),
                (
                    "Acceleration Oscillation",
                    "ACCELOSC_command=2",
                    "ACCELOSC_command=0",
                    10,
                ),
                ("Runaway", "ACCELRUNAWAY_command=1", "ACCELRUNAWAY_command=0", 5),
            ]

            for name, on, off, t in tests:
                tester.run_fault(name, on, off, t)

            tester.emergency_test()

        elif choice == 16:

            value = ask_float("East bias in meters (default 2): ", 2)

            tester.run_fault(
                "GNSS East Bias",
                "GNSS_EAST_BIAS_M_command={}".format(value),
                "GNSS_EAST_BIAS_M_command=0",
            )

        elif choice == 17:

            value = ask_float("North bias in meters (default 2): ", 2)

            tester.run_fault(
                "GNSS North Bias",
                "GNSS_NORTH_BIAS_M_command={}".format(value),
                "GNSS_NORTH_BIAS_M_command=0",
            )

        elif choice == 18:

            tester.run_fault(
                "GNSS Dropout",
                "GNSS_DROPOUT_command=1",
                "GNSS_DROPOUT_command=0",
            )

        elif choice == 19:

            tester.run_fault(
                "GNSS No-Fix Status",
                "GNSS_NO_FIX_command=1",
                "GNSS_NO_FIX_command=0",
            )

        elif choice == 20:

            value = int(ask_float("Drop every nth scan (default 5): ", 5))

            if value <= 0:
                print("The scan interval must be a positive integer.")
            else:
                tester.run_fault(
                    "LiDAR Periodic Scan Dropout",
                    "LIDAR_DROP_EVERY_N_command={}".format(value),
                    "LIDAR_DROP_EVERY_N_command=0",
                    10,
                )

        elif choice == 21:

            value = ask_float("Timestamp offset in seconds (default 0.1): ", 0.1)

            tester.run_fault(
                "LiDAR Timestamp Offset",
                "LIDAR_TIMESTAMP_OFFSET_S_command={}".format(value),
                "LIDAR_TIMESTAMP_OFFSET_S_command=0",
                10,
            )

        elif choice == 22:

            duration = ask_float("Duration in seconds (default 5): ", 5)
            tester.localization_input_loss_test(duration)

        else:

            print("Unknown selection.")
