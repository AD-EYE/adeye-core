#!/usr/bin/env python

import os  # to record rosbags using the command line, os is used to manage SIGINT
import subprocess  # for recording feature
import time  # to put timestamp in rosbag names
import re
from enum import Enum  # to make enumeration (in particular the features enumeration)

import rospkg  # to find path to AD-EYE package
import rospy  # for ROS
from std_msgs.msg import Bool
from std_msgs.msg import Int32
from std_msgs.msg import String
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Int8
from autoware_msgs.msg import VehicleStatus, VehicleCmd, ControlCommand


from FeatureControl import FeatureControl  # handles start and stop of features
from collections import OrderedDict  # to have the features ordered


## This class only contains the state machine for the manager.
# The state machine has four states defined in the States enum. The class listens to the topic containing the
# transition requests and performs the allowed transitions or prints an error message.
class ManagerStateMachine:
    # States of the machine
    class States(Enum):
        INITIALIZING_STATE = 0
        ENABLED_STATE = 1
        ENGAGED_STATE = 2
        FAULT_STATE = 3

    class VehicleStates:
        lamps = 0
        speed_gui = 0.0
        steer_gui = 0.0
        steer_being_kept = 0.0
        hl_gui = 0
        wheel_gui = 1
        accelerate_gui = 0.0

    current_state = States.INITIALIZING_STATE  # this is the current state of the state machine INITIALIZING_STATE

    ## The constructor
    #
    #@param self The object pointer
    def __init__(self):
        # Set up subscriber for registering state switch commands
        rospy.Subscriber("/initial_checks", Bool, self.initialChecksCallback)
        rospy.Subscriber("/activation_request", Bool, self.activationRequestCallback)
        rospy.Subscriber("/state_cmd", String, self.emergencyCallback)
        rospy.Subscriber("/vehicle_status", VehicleStatus, self.vehicleStatusCallback)
        rospy.Subscriber("/vehicle_commands", String, self.vehicleCommandCallback)
        #rospy.Subscriber("/vehicle_cmd", VehicleCmd, self.steerCmdCallback)


        self.send_state_pub = rospy.Publisher('/vehicle_status', VehicleStatus, queue_size=1)
        self.send_wheel_state_pub = rospy.Publisher('/vehicle_cmd', VehicleCmd, queue_size=1)
        self.send_vehicle_commands_pub = rospy.Publisher('/vehicle_commands', String, queue_size=1)


    ##Method getState
    #
    # Returns the state the machine is currently in
    #@param self The object pointer
    def getState(self):
        return self.current_state

    ##Method printRefusedRequest
    #
    # Help function for message display when invalid transition is requested
    #@param self The object pointer
    def printRefusedRequest(self):
        rospy.loginfo("Request refused. Manager will stay in" + self.getState().name)

    ##Method initialChecksCallback
    #
    # Callback to switch from initializing to enabled, listens to /initial_checks
    #@param self The object pointer
    #@param msg A Boolean from the /initial_checks topic
    def initialChecksCallback(self, msg):
        if msg.data and self.current_state == self.States.INITIALIZING_STATE:
            rospy.loginfo("Entering Enabled state")
            self.current_state = self.States.ENABLED_STATE
            rospy.loginfo("System can be activated")
        else:
            self.printRefusedRequest()

    ##Method activationRequestCallback
    #
    # Callback to switch from enabled to engaged or the other way, listens to /activation_request
    #@param self The object pointer
    #@param msg A Boolean from the /activation_request topic
    def activationRequestCallback(self, msg):
        if msg.data and self.current_state == self.States.ENABLED_STATE:
            rospy.loginfo("Entering Engaged state")
            self.current_state = self.States.ENGAGED_STATE
        elif not msg.data and self.current_state == self.States.ENGAGED_STATE:  # deactivation
            rospy.loginfo("Entering Enabled state from Engaged")
            self.current_state = self.States.ENABLED_STATE
        else:
            self.printRefusedRequest()

    ##Method emergencyCallback
    #
    # Callback to switch to the fault state from any state, listens to /state_cmd
    #@param self The object pointer
    #@param msg A String from the /state_cmd topic
    def emergencyCallback(self, msg):
        if msg.data == "emergency":
            rospy.loginfo("Entering Fault state")
            self.current_state = self.States.FAULT_STATE
        elif msg.data == "return_to_ready":
            rospy.loginfo("Entering Enabled state from Fault")
            self.current_state = self.States.ENABLED_STATE
        elif msg.data == "return_from_emergency":
            rospy.loginfo("Entering Initializing state from Fault")
            self.current_state = self.States.INITIALIZING_STATE
        else:
            pass

    ##Method vehicleStatusCallback
    #
    # Callback to update hazard lights and set speed to zero when in fault state, listens to /vehicle_status
    #@param self The object pointer
    #@param msg A VehicleStatus from the /vehicle_status topic
    def vehicleStatusCallback(self, msg):
        if self.current_state == self.States.FAULT_STATE:
            """
             Update hazard lamp
            if msg.lamp != 1:
                msg.lamp = 1
                self.send_state_pub.publish(msg)
                rospy.loginfo("Set lamp to emergency")
                self.VehicleStates.lamps = msg.lamp
            else:
                pass
                 """
            if self.VehicleStates.wheel_gui == 0:
                # Update current speed (m/s)
                if msg.speed != self.VehicleStates.speed_gui:
                    # Checking if speed is effectively zero (allowing a small threshold for floating point errors)
                    if self.VehicleStates.speed_gui <= 0.1:
                        msg.speed = 0.0 # Explicitly set speed to zero
                        self.send_state_pub.publish(msg)
                        rospy.loginfo("Set speed to zero because of wheel lock from GUI:")
                        rospy.loginfo(msg.speed)
            elif self.VehicleStates.wheel_gui == 1:
                    if msg.speed != self.VehicleStates.speed_gui:
                        msg.speed = self.VehicleStates.speed_gui
                        self.send_state_pub.publish(msg)            # If resumed, update speed
                        rospy.loginfo("Wheel lock turned off from GUI, resuming normal speed calculations:")
                        rospy.loginfo(msg.speed)

            if self.VehicleStates.speed_gui == 0:
                if msg.speed != 0.0:
                    msg.speed = 0.0 # Explicitly set speed to zero
                    self.send_state_pub.publish(msg)
                    rospy.loginfo("Set speed to zero because of explicit wheel lock from GUI:")
                    rospy.loginfo(msg.speed)

            if self.VehicleStates.hl_gui == 1:
                # Update hazard lamp
                if msg.lamp != 1:
                    msg.lamp = 1
                    self.send_state_pub.publish(msg)
                    rospy.loginfo("Set lamp to emergency from GUI")
            elif self.VehicleStates.hl_gui == 0:
                # Update hazard lamp
                if msg.lamp != 0:
                    msg.lamp = 0
                    self.send_state_pub.publish(msg)
                    rospy.loginfo("Set lamp to normal from GUI")


            if self.VehicleStates.wheel_gui == 0:
            # Update current steering angle (radians) if speed if zero
                if msg.speed == 0.0 and self.VehicleStates.steer_being_kept == 0.0:
                    self.VehicleStates.steer_being_kept = self.VehicleStates.steer_gui
                    msg.angle = self.VehicleStates.steer_being_kept  # If stopped, get the last steering angle
                    self.send_state_pub.publish(msg)
                    rospy.loginfo("Getting last wheel angle:")
                    rospy.loginfo(msg.angle)
                elif msg.speed == 0.0 and self.VehicleStates.steer_being_kept != 0.0:
                    if  self.VehicleStates.steer_gui != self.VehicleStates.steer_being_kept:
                        self.VehicleStates.steer_gui = self.VehicleStates.steer_being_kept
                        msg.angle = self.VehicleStates.steer_being_kept
                        self.send_state_pub.publish(msg)
                        self.send_vehicle_commands_pub.publish("STEERING_command=" + str(self.VehicleStates.steer_being_kept))
                        rospy.loginfo("Still keeping wheel in place from GUI")
                        rospy.loginfo(msg.angle)
            elif self.VehicleStates.wheel_gui == 1:
                    if msg.angle != self.VehicleStates.steer_gui:
                        msg.angle = self.VehicleStates.steer_gui  # If resumed, update steering angle
                        self.send_state_pub.publish(msg)
                        rospy.loginfo("Stopped keeping wheel in place from GUI")
                        rospy.loginfo(msg.angle)
                        self.VehicleStates.steer_being_kept = 0.0

        elif self.current_state == self.States.ENABLED_STATE or self.current_state == self.States.INITIALIZING_STATE:
             # Update hazard lamp
            if msg.lamp == 1:
                msg.lamp = 0
                self.send_state_pub.publish(msg)
                rospy.loginfo("Set lamp to normal because of enabled or initialized state")
                self.VehicleStates.lamps = msg.lamp

            if msg.speed != self.VehicleStates.speed_gui:
                    msg.speed = self.VehicleStates.speed_gui
                    rospy.loginfo("Controlling speed from GUI")
                    rospy.loginfo(msg.speed)
            if msg.angle != self.VehicleStates.steer_gui:
                        msg.angle = self.VehicleStates.steer_gui  # If resumed, update steering angle
                        self.send_state_pub.publish(msg)
                        rospy.loginfo("Controlling steer from GUI")
                        rospy.loginfo(msg.angle)

        """
        else:
             self.printRefusedRequest()
        """

    ##Method steerCmdCallback
    #
    # Callback to lock the steering wheel position when in fault state, listens to /vehicle_cmd
    #@param self The object pointer
    #@param msg A SteerCmd from the /vehicle_cmd topic
    """ def steerCmdCallback(self, msg):
        if self.current_state == self.States.FAULT_STATE:
            # Update current steering angle (radians)
            if self.VehicleStates.steer != msg.steer and self.VehicleStates.speed == 0.0:
                self.VehicleStates.steer = msg.steer
                msg.steer = self.VehicleStates.steer  # If stopped, maintain the last steering angle
                self.send_wheel_state_pub.publish(msg)
                rospy.loginfo("Keeping wheel in place")
        el

            if self.VehicleStates.wheel_gui == 0:
            # Update current steering angle (radians) if speed if zero
                if self.VehicleStates.steer_gui != msg.ControlCommand.steering_angle and self.VehicleStates.speed_gui == 0.0:
                    msg.ControlCommand.steering_angle = self.VehicleStates.steer_gui  # If stopped, maintain the last steering angle
                    self.send_wheel_state_pub.publish(msg)
                    rospy.loginfo("Keeping wheel in place on from GUI")
                    rospy.loginfo(msg.ControlCommand.steering_angle)
                else:
                    rospy.loginfo("Still keeping wheel in place on from GUI")
                    rospy.loginfo(msg.ControlCommand.steering_angle)
            elif self.VehicleStates.wheel_gui == 1:
                    msg.ControlCommand.steering_angle = self.VehicleStates.steer_gui  # If resumed, update steering angle
                    self.send_wheel_state_pub.publish(msg)
                    rospy.loginfo("Stop keeping wheel in place from GUI")
                    rospy.loginfo(msg.ControlCommand.steering_angle) """
       # else:
         #   self.printRefusedRequest()

    ##Method vehicleCommandCallback
    #
    # Callback to lock the steering wheel position or turn on hazard lights from GUI, listens to /vehicle_commands
    #@param self The object pointer
    #@param msg A String from the /vehicle_commands topic
    def vehicleCommandCallback(self, msg):
        if msg.data == "HL_command=1":
            # Update hazard lights from gui
            if self.VehicleStates.hl_gui != 1:
                self.VehicleStates.hl_gui = 1
                rospy.loginfo("Hazard lights on from GUI")
        elif msg.data == "HL_command=0":
            # Update hazard lights from gui
            if self.VehicleStates.hl_gui != 0:
                self.VehicleStates.hl_gui = 0
                rospy.loginfo("Hazard lights off from GUI")
        elif msg.data == "WLOCK_command=0":
            # Update wheel lock from gui
            if self.VehicleStates.wheel_gui != 0:
                self.VehicleStates.wheel_gui = 0
                rospy.loginfo("Wheel lock on from GUI")
        elif msg.data == "WLOCK_command=1":
            # Update wheel lock from gui
            if self.VehicleStates.wheel_gui != 1:
                self.VehicleStates.wheel_gui = 1
                rospy.loginfo("Wheel lock off from GUI")
        if msg.data.find("ACCELERATE_command=") != -1:
            # Update accelerate from gui
            a = re.findall(r'(-?\d+\.?\d*)', msg.data)
            rospy.loginfo(a)
            b = float(a[0])
            if self.VehicleStates.speed_gui != b:
                self.VehicleStates.speed_gui = b
                rospy.loginfo("Setting accelerate from GUI")
                rospy.loginfo(self.VehicleStates.speed_gui)

        if msg.data.find("STEERING_command=") != -1:
            a = re.findall(r'(-?\d+\.?\d*)', msg.data)
            rospy.loginfo(a)
            b = float(a[0])
            if self.VehicleStates.steer_gui != b:
                self.VehicleStates.steer_gui = b
                # Update steer from gui
                rospy.loginfo("Setting steer from GUI")
                rospy.loginfo(self.VehicleStates.steer_gui)



## Feature wrapper for convenience of the ManagerFeaturesHandler class.
class Feature:
    name = ""
    path = ""
    start_delay = 0
    stop_delay = 0

    ## The constructor
    #
    # @param self The object pointer
    #  @param name Feature name
    #  @param path Path to feature launch file
    #  @param start_delay How long should we wait before starting the feature
    #  @param stop_delay How long should we wait before stopping the feature
    def __init__(self, name, path, start_delay, stop_delay):
        self.name = name
        self.path = path
        self.start_delay = start_delay
        self.stop_delay = stop_delay

    ##Method createFeatureControl
    #
    #A method that creates the FeatureControl object which will be used to start and stop the features
    def createFeatureControl(self):
        self.featureControl = FeatureControl(self.path, self.name, self.start_delay, self.stop_delay)

## A class handling all the features as well as performing the launch files paths construction
class ManagerFeaturesHandler:
    features = OrderedDict()
    # Ordered dictionary object                 = Feature(Feature_name,                    launch_file_name,              start_delay, stop_delay)
    features["Recording"] = Feature("Recording", "", 0, 0)
    features["Map"] = Feature("Map", "my_map.launch", 8, 0)
    features["Sensing"] = Feature("Sensing", "my_sensing.launch", 10, 0)
    features["Localization"] = Feature("Localization", "my_localization.launch", 8, 5)
    features["Fake_Localization"] = Feature("Fake_Localization", "my_fake_localization.launch", 0, 0)
    features["Detection"] = Feature("Detection", "my_detection.launch", 0, 5)
    features["Mission_Planning"] = Feature("Mission_Planning", "my_mission_planning.launch", 2, 5)
    features["Motion_Planning"] = Feature("Motion_Planning", "my_motion_planning.launch", 0, 5)
    features["Switch"] = Feature("Switch", "switch.launch", 0, 0)
    features["SSMP"] = Feature("SSMP", "SSMP.launch", 0, 0)
    features["Rviz"] = Feature("Rviz", "my_rviz.launch", 0, 0)
    features["Experiment_specific_recording"] = Feature("Experiment_specific_recording", "", 0, 0)

    ## The constructor
    #
    #@param self The object pointer
    def __init__(self):
        self.createLaunchPaths()
        self.createFeaturesControls()

    ##Method createLaunchPaths
    #
    #Constructs the launch files paths from their names and whether we use Test Automation or not
    #@param self The object pointer
    def createLaunchPaths(self):
        # Basic folder locations
        rospack = rospkg.RosPack()
        adeye_package_path = rospack.get_path('adeye') + "/"
        launch_folder = "launch/"

        if rospy.get_param("test_automation", False):
            launch_folder = "modified_launch_files/"
            for key in self.features:
                self.features[key].path = "rp_" + self.features[key].path

        for key in self.features:
            self.features[key].path = adeye_package_path + launch_folder + self.features[key].path

        self.features["Experiment_specific_recording"].path = "/home/adeye/AD-EYE_Core/AD-EYE/ROS_Packages/src/AD-EYE" \
                                                              "/launch/ExperimentFaults.launch"
        self.features["Recording"].path = ""

    ##Method createFeaturesControls
    #
    #A class that creates the FeaturesControl objects, must be called after the launch paths are constructed
    #@param self The object pointer
    def createFeaturesControls(self):
        for key in self.features:
            self.features[key].createFeatureControl()


##Manager Class
#
#This class :
#
#Checks if safety channel is active, if not prints warning
#
#Checks state of the state machine and updates current features accordingly
#
#Checks if the list of active features has changed
#
#Publishes the state machine state (for GUI)
#
#Publishes the current active features (for GUI)
class Manager:
    INITIALIZING_DEFAULT_FEATURES = [
        # "Recording",
        "Map",
        "Sensing",
        "Localization",
        # "Fake_Localization",
        #"Detection",
        "Mission_Planning",
        "Motion_Planning",
        "Switch",
        #"SSMP",
        # "Rviz",
        # "Experiment_specific_recording"
    ]
    ENABLED_DEFAULT_FEATURES = [
        # "Recording",
        "Map",
        "Sensing",
        "Localization",
        # "Fake_Localization",
        #"Detection",
        "Mission_Planning",
        "Motion_Planning",
        "Switch",
        #"SSMP",
        # "Rviz",
        # "Experiment_specific_recording"
    ]
    ENGAGED_DEFAULT_FEATURES =[
        # "Recording",
        "Map",
        "Sensing",
        "Localization",
        # "Fake_Localization",
        #"Detection",
        "Mission_Planning",
        "Motion_Planning",
        "Switch",
        #"SSMP",
        # "Rviz",
        # "Experiment_specific_recording"
    ]
    FAULT_DEFAULT_FEATURES = [
        # "Recording",
        "Map",
        "Sensing",
         "Localization",
        # "Fake_Localization",
        # "Detection",
         "Mission_Planning",
         "Motion_Planning",
        "Switch",
        #"SSMP",
        # "Rviz",
        # "Experiment_specific_recording"
    ]
    INITIALIZING_ALLOWED_FEATURES = [
        # "Recording",
        "Map",
        "Sensing",
        "Localization",
        # "Fake_Localization",
        #"Detection",
        "Mission_Planning",
        "Motion_Planning",
        "Switch",
        #"SSMP",
        # "Rviz",
        # "Experiment_specific_recording"
    ]
    ENABLED_ALLOWED_FEATURES = [
        # "Recording",
        "Map",
        "Sensing",
        "Localization",
        # "Fake_Localization",
        #"Detection",
        "Mission_Planning",
        "Motion_Planning",
        "Switch",
        #"SSMP",
        # "Rviz",
        # "Experiment_specific_recording"
    ]
    ENGAGED_ALLOWED_FEATURES = [
        # "Recording",
        "Map",
        "Sensing",
        "Localization",
        # "Fake_Localization",
        #"Detection",
        "Mission_Planning",
        "Motion_Planning",
        "Switch",
        #"SSMP",
        # "Rviz",
        # "Experiment_specific_recording"
    ]
    FAULT_ALLOWED_FEATURES = [
        # "Recording",
        "Map",
        "Sensing",
        "Localization",
        # "Fake_Localization",
        # "Detection",
        "Mission_Planning",
        "Motion_Planning",
        "Switch",
        #"SSMP",
        # "Rviz",
        # "Experiment_specific_recording"
    ]
    previous_features = []
    current_features = INITIALIZING_DEFAULT_FEATURES

    # Rosbag related constants
    ROSBAG_PATH = "/recording" + str(time.time()) + ".bag"  # ~ is added as a prefix, name of the bag
    ROSBAG_COMMAND = "rosbag record -a -O ~" + ROSBAG_PATH + " __name:=rosbag_recorder"  # command to start the rosbag

    # To output an error message when safety channel is not running
    last_switch_time = 0
    last_switch_time_initialized = False
    SWITCH_TIME_THRESHOLD = 3  # after this amount of time (sec) the manager will write an error message if nothing is received from the safety supervisor

    ##The constructor
    #
    #@param self The object pointer
    def __init__(self):
        self.manager_state_machine = ManagerStateMachine()
        self.current_state = self.manager_state_machine.States.INITIALIZING_STATE

        self.manager_features_handler = ManagerFeaturesHandler()

        rospy.Subscriber("/Features_state", Int32MultiArray, self.featuresRequestCallback)
        rospy.Subscriber("/switch_command", Int32, self.switchCallback)  # to check if safety channel is still alive
        self.state_pub = rospy.Publisher('manager/state', Int8, queue_size=1)  # for GUI
        self.features_pub = rospy.Publisher('manager/features', Int32MultiArray, queue_size=1)  # for GUI
        self.switch_request_pub = rospy.Publisher('/safety_channel/switch_request', Int32, queue_size=1)  # for GUI



    ##The main loop
    #@param self The object pointer
    def run(self):
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.runOnce()
            rate.sleep()

    ##Method runOnce
    #Contains what should be done in one iteration of the main loop
    #@param self The object pointer
    def runOnce(self):
        self.checkSafetyChannel()  # checks if safety channel is active, if not prints warning
        self.checkManagerState()  # check state of the state machine and updates current features accordingly
        if self.current_features != self.previous_features:  # checks if the list of active features has changed
            self.startAndStopFeatures()
        self.state_pub.publish(self.manager_state_machine.getState().value)  # publish the state machine state (for GUI)
        self.publishBooleanListActiveFeatures()  # publish the current active features (for GUI)

    ##Callback method listening to the features requests (features that we want to activate/deactivate)
    #@param self The object pointer
    #@param msg A Int32MultiArray message from the /Features_state topic
    def featuresRequestCallback(self, msg):
        message_features = []
        allowed_features = self.getAllowedFeatures()
        for i in range(len(msg.data)):
            if msg.data[i] == 1:
                if self.manager_features_handler.features.keys()[i] in allowed_features:
                    message_features.append(self.manager_features_handler.features.keys()[i])
        self.current_features = message_features

    ##Callback method listening to switch messages, to make sure the safety channel is still active
    #@param self The object pointer
    #@param msg An Int32 message from the /switch_command topic
    def switchCallback(self, msg):
        self.last_switch_time = rospy.Time.now()
        self.last_switch_time_initialized = True

    ##A method to checks if the a message from the safety channel has been received recently, if not prints warning
    #@param self The object pointer
    def checkSafetyChannel(self):
        if self.last_switch_time_initialized:
            if (rospy.Time.now() - self.last_switch_time).to_sec() > self.SWITCH_TIME_THRESHOLD:
                rospy.logerr("No message from the safety channel")

    ##A method that gets the list of allowed features based on the manager's state
    #@param self The object pointer
    def getAllowedFeatures(self):
        state = self.manager_state_machine.getState()
        if state == self.manager_state_machine.States.INITIALIZING_STATE:
            return self.INITIALIZING_ALLOWED_FEATURES
        elif state == self.manager_state_machine.States.ENABLED_STATE:
            return self.ENABLED_ALLOWED_FEATURES
        elif state == self.manager_state_machine.States.ENGAGED_STATE:
            return self.ENGAGED_ALLOWED_FEATURES
        elif state == self.manager_state_machine.States.FAULT_STATE:
            return self.FAULT_ALLOWED_FEATURES

    ##A method that checks if the manager state has changed. If so, updates the current features list
    #@param self The object pointer
    def checkManagerState(self):
        state = self.manager_state_machine.getState()
        if state != self.current_state:  # the state has changed since last iteration
            self.current_state = state
            if state == self.manager_state_machine.States.INITIALIZING_STATE:
                msg = Int32()
                msg.data = 0
                self.switch_request_pub.publish(msg)
                self.current_features = self.INITIALIZING_DEFAULT_FEATURES
                #self.send_state.publish("emergency")
            elif state == self.manager_state_machine.States.ENABLED_STATE:
                msg = Int32()
                msg.data = 1
                self.switch_request_pub.publish(msg)
                self.current_features = self.ENABLED_DEFAULT_FEATURES
                #self.send_state.publish("emergency")
            elif state == self.manager_state_machine.States.ENGAGED_STATE:
                msg = Int32()
                msg.data = 2
                self.switch_request_pub.publish(msg)
                self.current_features = self.ENGAGED_DEFAULT_FEATURES
                #self.send_state.publish("emergency")
            elif state == self.manager_state_machine.States.FAULT_STATE:
                msg = Int32()
                msg.data = 3
                self.switch_request_pub.publish(msg)  # when we enter fault state we first force the switch to safety channel
                self.current_features = self.FAULT_DEFAULT_FEATURES


    ##A method that checks if a feature is now in the list of features that should be active but was not in the previous iteration
    #@param self The object pointer
    #@param feature_name A String
    def isFeatureJustActivated(self, feature_name):
        return feature_name in self.current_features and feature_name not in self.previous_features

    ##A method that checks if a feature is not in the list of features that should be active but was in the previous iteration
    #@param self The object pointer
    #@param feature_name A String
    def isFeatureJustDeactivated(self, feature_name):
        return feature_name not in self.current_features and feature_name in self.previous_features

    ##A method that starts the individual features based on the current feature list
    #@param self The object pointer
    def startAndStopFeatures(self):
        for feature_name in self.manager_features_handler.features:
            if feature_name == "Recording":  # recording needs special case as it is not a launch file like other features
                if self.isFeatureJustActivated(feature_name):
                    self.startRecording()
                elif self.isFeatureJustDeactivated(feature_name):
                    self.stopRecording()
            else:  # "normal" features
                try:  # to not kill the manager when a launch is malformed (in that case an exception is thrown)
                    if self.isFeatureJustActivated(feature_name):
                        self.manager_features_handler.features[feature_name].featureControl.start()
                    elif self.isFeatureJustDeactivated(feature_name):
                        self.manager_features_handler.features[feature_name].featureControl.stop()
                    else:
                        pass  # nothing to do, feature stays enable/disabled
                except Exception as exc:
                    rospy.logerr("Manager failed to start " + feature_name + ": " + str(exc))
        self.previous_features = self.current_features

    ##A method that starts the rosbag recording process
    #@param self The object pointer
    def startRecording(self):
        subprocess.Popen(self.ROSBAG_COMMAND, shell=True, executable='/bin/bash')

    ##A method that starts the scripts for stopping the rosbag recording
    #@param self The object pointer
    def stopRecording(self):
        rospack = rospkg.RosPack()
        adeye_package_location = rospack.get_path('adeye') + "/"
        subprocess.Popen(
            "xterm -hold -e bash " + adeye_package_location + "/sh/rosbagStop.sh ~/" + self.ROSBAG_PATH,
            shell=True, preexec_fn=os.setpgrp, executable='/bin/bash')

    def getBooleanListActiveFeatures(self):
        state_array = Int32MultiArray()
        for feature in self.manager_features_handler.features:
            if feature in self.current_features:
                state_array.data.append(1)
            else:
                state_array.data.append(0)
        return state_array


    def publishBooleanListActiveFeatures(self):
        self.features_pub.publish(self.getBooleanListActiveFeatures())

if __name__ == '__main__':
    rospy.init_node('AD-EYE_Manager')
    rospy.loginfo("ADEYE Manager: Started")
    manager = Manager()
    manager.run()
