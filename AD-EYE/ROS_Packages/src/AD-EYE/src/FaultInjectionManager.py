#!/usr/bin/env python

import re
import rospy
import math
import datetime
import random
from std_msgs.msg import String
from autoware_msgs.msg import VehicleStatus, VehicleCmd


class FaultInjectionManager:

    def __init__(self, manager_state_machine):

        self.fault_log = open("/tmp/adeye_fault_log.txt", "a")

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
        lamps = 0
        speed_gui = 0.0
        steer_gui = 0.0
        # changed from 0.0
        steer_being_kept = None
        hl_gui = 0
        wheel_gui = 1
        accelerate_gui = 0.0

        # Steering faults
        steer_offset_gui = 0.0
        steer_freeze_gui = 0
        steer_frozen_value = None
        steer_saturation_gui = 0.0
        steer_oscillation_gui = 0.0
        steer_random_gui = 0

        # Accelerating faults
        accelerate_offset_gui = 0.0
        accelerate_freeze_gui = 0
        accelerate_frozen_value = None
        accelerate_saturation_gui = 0.0
        accelerate_oscillation_gui = 0.0
        accelerate_runaway_gui = 0

        current_speed = 0.0
        current_angle = 0.0
        current_lamp = 0

    def logFaultDebug(self, fault, value):

        # rostopic echo /adeye/fault_log

        msg = fault + ": " + str(value)

        self.fault_debug_pub.publish(msg)

        rospy.loginfo(msg)

    def logFault(self, fault, value):

        timestamp = datetime.datetime.now()

        line = str(timestamp) + " | " + fault + " | " + str(value) + "\n"

        self.fault_log.write(line)
        self.fault_log.flush()

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

        self.VehicleStates.current_speed = msg.speed
        self.VehicleStates.current_angle = msg.angle
        #######################################################################
        # Wheel lock
        #######################################################################

        if self.VehicleStates.wheel_gui == 0:

            if abs(msg.speed) < 0.1:

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

    ##########################################################################
    # VEHICLE COMMAND CALLBACK
    ##########################################################################

    def vehicleCommandCallback(self, msg):

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

        if msg.data.find("ACCELERATE_command=") != -1:

            a = re.findall(r"(-?\d+\.?\d*)", msg.data)

            rospy.loginfo(a)

            b = float(a[0])

            if self.VehicleStates.speed_gui != b:

                self.VehicleStates.speed_gui = b

                rospy.loginfo("Setting accelerate from GUI")
                rospy.loginfo(self.VehicleStates.speed_gui)

        if msg.data.find("STEERING_command=") != -1:

            a = re.findall(r"(-?\d+\.?\d*)", msg.data)

            rospy.loginfo(a)

            b = float(a[0])

            if self.VehicleStates.steer_gui != b:

                self.VehicleStates.steer_gui = b

                rospy.loginfo("Setting steer from GUI")
                rospy.loginfo(self.VehicleStates.steer_gui)

        if msg.data.find("STEEROFFSET_command=") != -1:

            a = re.findall(r"(-?\d+\.?\d*)", msg.data)

            rospy.loginfo(a)

            b = float(a[0])

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

        if msg.data.find("STEERSAT_command=") != -1:

            a = re.findall(r"(-?\d+\.?\d*)", msg.data)

            rospy.loginfo(a)

            b = float(a[0])

            if self.VehicleStates.steer_saturation_gui != b:

                self.VehicleStates.steer_saturation_gui = b

                rospy.loginfo("Setting steering saturation from GUI")

                rospy.loginfo(self.VehicleStates.steer_saturation_gui)

        if msg.data.find("STEEROSC_command=") != -1:

            a = re.findall(r"(-?\d+\.?\d*)", msg.data)

            rospy.loginfo(a)

            b = float(a[0])

            if self.VehicleStates.steer_oscillation_gui != b:

                self.VehicleStates.steer_oscillation_gui = b

                rospy.loginfo("Setting steering oscillation from GUI")

                rospy.loginfo(self.VehicleStates.steer_oscillation_gui)

        # Acceleration offset

        if msg.data.find("ACCELOFFSET_command=") != -1:

            a = re.findall(r"(-?\d+\.?\d*)", msg.data)

            rospy.loginfo(a)

            b = float(a[0])

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

        if msg.data.find("ACCELSAT_command=") != -1:

            a = re.findall(r"(-?\d+\.?\d*)", msg.data)

            rospy.loginfo(a)

            b = float(a[0])

            if self.VehicleStates.accelerate_saturation_gui != b:

                self.VehicleStates.accelerate_saturation_gui = b

                rospy.loginfo("Setting acceleration saturation")

                rospy.loginfo(self.VehicleStates.accelerate_saturation_gui)

        # Acceleration oscillation

        if msg.data.find("ACCELOSC_command=") != -1:

            a = re.findall(r"(-?\d+\.?\d*)", msg.data)

            rospy.loginfo(a)

            b = float(a[0])

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

        else:

            if msg.lamp_cmd.l != 0 or msg.lamp_cmd.r != 0:

                msg.lamp_cmd.l = 0
                msg.lamp_cmd.r = 0

                modified = True

                self.logFault("HAZARD_LIGHTS", "OFF")

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

            if msg.ctrl_cmd.steering_angle > limit:

                msg.ctrl_cmd.steering_angle = limit

                modified = True

            elif msg.ctrl_cmd.steering_angle < -limit:

                msg.ctrl_cmd.steering_angle = -limit

                modified = True

            if modified:

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

            msg.ctrl_cmd.linear_acceleration += (
                self.VehicleStates.accelerate_oscillation_gui
                * math.sin(rospy.get_time())
            )

            msg.accel_cmd.accel += (
                self.VehicleStates.accelerate_oscillation_gui
                * math.sin(rospy.get_time())
            )

            modified = True

            self.logFault(
                "ACCELERATION_OSCILLATION",
                self.VehicleStates.accelerate_oscillation_gui,
            )

        #######################################################################
        # Publish modified command
        #######################################################################

        if modified:

            self.republishing_vehicle_cmd = True

            self.send_wheel_state_pub.publish(msg)

            self.republishing_vehicle_cmd = False
