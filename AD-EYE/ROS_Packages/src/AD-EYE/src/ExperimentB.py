#!/usr/bin/env python

import roslib
import os
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from autoware_msgs.msg import NDTStat
import tf


if os.path.isdir('/home/adeye/Experiment_Results/ExperimentB/') == False : # checks if the folder exists and creates it if not
    os.mkdir('/home/adeye/Experiment_Results/ExperimentB/')
NumberFound = False
k=1
while NumberFound == False :
    if os.path.isfile('/home/adeye/Experiment_Results/ExperimentB/ExperimentB'+str(k)+'.csv')==True: # creates a file for each experiment
        k+=1
    else :
        file = open('/home/adeye/Experiment_Results/ExperimentB/ExperimentB'+str(k)+'.csv','a')
        NumberFound=True

##@var STOP_DISTANCE_SQUARRED
#
#We stop the experiment when closer than this value and not moving
STOP_DISTANCE_SQUARRED = 900 

##A class to run an record the experiment B, to check the behavior of the vehicle when it's reaching its goal.
#We study the influence of rain on the lidar on various terrains (semi-urban, forest, and urban area) by storing
#in files the true position and the estimated position of the ego vehicle.
class ExperimentBRecorder:

    start = False
    firstStore = True

    groud_truth_poses  = []
    esimated_poses  = []
    localization_stats = []
    new_ground_truth = False #this aims to run storeData() only when new data are published on both nodes /gnss_pose and /ndt_pose
    new_estimate = False
    goal_x = 0
    goal_y = 0

    #The constructor
    #
    #@param self The object pointer
    def __init__(self):
        rospy.Subscriber("/ndt_stat", NDTStat, self.localizationStatCallback)
        rospy.Subscriber("/ndt_pose", PoseStamped, self.estimatedPoseCallback)
        rospy.Subscriber("/gnss_pose", PoseStamped, self.groundTruthPoseCallback)
        rospy.Subscriber("/current_velocity", TwistStamped, self.egoSpeedCallback)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goalCallback)
        self.stop_pub = rospy.Publisher("/simulink/stop_experiment",Bool, queue_size = 1) # stop_publisher


    ##A method to update the list of positions and orientations for groundTruth
    #@param self The object pointer
    #@param data A PoseStamped message from the /gnss_pose topic
    def groundTruthPoseCallback(self, data):
        Px = data.pose.position.x
        Py = data.pose.position.y
        Pz = data.pose.position.z
        T = self.getTime()
        L = [Px,Py,Pz,T,data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
        self.groud_truth_poses.append(L)
        self.new_ground_truth = True
        if (self.start == True) and (self.new_ground_truth==True) and (self.new_estimate == True) :
            self.storeData()

    ##A method to update the list of estimated positions and orientations
    #@param self The object pointer
    #@param data A PoseStamped message from the /ndt_pose topic
    def estimatedPoseCallback(self, data):
        Px = data.pose.position.x
        Py = data.pose.position.y
        Pz = data.pose.position.z
        T = self.getTime()
        L = [Px,Py,Pz,T,data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
        self.esimated_poses.append(L)
        self.new_estimate = True
        if (self.start == True) and (self.new_ground_truth==True) and (self.new_estimate == True) :
            self.storeData()

    ##A method to update the list localization_stats
    #@param self The object pointer
    #@param data A NDTStat message from the /ndt_stat topic
    def localizationStatCallback(self, data):
        itterations = data.iteration
        score = data.score
        exe_time = data.exe_time
        T = self.getTime()
        L = [itterations,T,score,exe_time]
        self.localization_stats.append(L)

    ##A method to start or stop the experiment according to the speed and the distance to the goal
    #@param self The object pointer
    #@param data A TwistStamped message from the /current_velocity topic
    def egoSpeedCallback (self, data):
        Sp = data.twist.linear.x
        if self.start == False :
            if Sp > 0.0 :
                self.start = True
        else :
            if Sp == 0.0 and len(self.groud_truth_poses)>0 and self.distanceToGoal()<STOP_DISTANCE_SQUARRED:
                self.start = False
                self.stop_pub.publish(True) # stop_publisher

    ##A method to get the goal's coordinates
    #@param self The object pointer
    #@param data A PoseStamped message from the /move_base_simple/goal topic
    def goalCallback(self, data):
        self.goal_x = data.pose.position.x
        self.goal_y = data.pose.position.y


    ##A method to calculate the euclidean distance to the goal
    #@param self The object pointer
    def distanceToGoal(self):
        x,y = self.groud_truth_poses[-1][0],self.groud_truth_poses[-1][1]
        return (x - self.goal_x) ** 2 + (y - self.goal_y) ** 2


    ##A method to get the time
    #@param self The object pointer
    def getTime(self):
        now = rospy.get_rostime()
        return now.secs + now.nsecs * 10**9


    ##A method to write the experiment's parameters in the correct ExperimentB.csv file
    #@param self The object pointer
    def writeParameter(self):
        if self.firstStore :
            self.firstStore = False
            max_velocity = 0
            if rospy.has_param("adeye/motion_planning/op_common_params/maxVelocity"):
                max_velocity = float(rospy.get_param("adeye/motion_planning/op_common_params/maxVelocity"))
            file.write("Set speed ,"+str(max_velocity)+" , ")
            rain_intensity = rospy.get_param("/simulink/rain_intensity")
            file.write("Set rain intensity ,"+str(rain_intensity)+" , ")
            reflectivity = rospy.get_param("/simulink/reflectivity")
            file.write("Set reflectivity ,"+str(reflectivity)+" , ")
            file.write("goal x ,"+str(self.goal_x)+" , ")
            file.write("goal y ,"+str(self.goal_y)+" \n")


    ##A method to write in the correct file the true and estimated position and orientation, 
    ##the localization number of iterations, score, and execution time
    #@param self The object pointer
    def storeData (self):
        self.new_estimate = False # when this function is called, we "reset" the variables that indicates if new data is written or not
        self.new_ground_truth = False

        self.writeParameter()

        stat_len = len(self.localization_stats)-1
        ground_truth_len = len(self.groud_truth_poses)-1
        estimates_len = len(self.esimated_poses)-1
        if (stat_len>0) and (ground_truth_len)>0 and (estimates_len)>0:
            ground_truth_index = ground_truth_len
            estimate_index = estimates_len
            stat_index = stat_len


            xg,yg,zg = self.groud_truth_poses[ground_truth_index][0],self.groud_truth_poses[ground_truth_index][1],self.groud_truth_poses[ground_truth_index][2]
            oxg,oyg,ozg,owg = self.groud_truth_poses[ground_truth_index][4], self.groud_truth_poses[ground_truth_index][5], self.groud_truth_poses[ground_truth_index][6], self.groud_truth_poses[ground_truth_index][7]
            xe,ye,ze = self.esimated_poses[estimate_index][0],self.esimated_poses[estimate_index][1],self.esimated_poses[estimate_index][2]
            oxe,oye,oze,owe = self.esimated_poses[estimate_index][4], self.esimated_poses[estimate_index][5], self.esimated_poses[estimate_index][6], self.esimated_poses[estimate_index][7]
            nb_iter = self.localization_stats[stat_index][0]


            file.write("Ground Truth Pose position(x;y;z);quaternion(x;y;z;w), "+str(xg)+" , "+str(yg)+" , "+str(zg)+" , "+str(oxg)+" , "+str(oyg)+" , "+str(ozg)+" , "+str(owg)+" , ")
            file.write("Estimated Pose position(x;y;z);quaternion(x;y;z;w), "+str(xe)+" , "+str(ye)+" , "+str(ze)+" , "+str(oxe)+" , "+str(oye)+" , "+str(oze)+" , "+str(owe)+" , ")
            file.write("Localization, nb of iterrations , "+str(nb_iter)+" , ")
            file.write("Localization score , "+str(self.localization_stats[stat_index][2])+" , ")
            file.write("Localization execution time , "+str(self.localization_stats[stat_index][3]))

            file.write("\n")





# subscribes to the topics
if __name__ == '__main__':
    rospy.init_node('ExperimentB',anonymous = True)
    ExperimentBRecorder()
    rospy.spin()
