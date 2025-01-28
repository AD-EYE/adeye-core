#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseWithCovarianceStamped

def callback(msg):
    # Update the header timestamp with the current ROS time
    msg.header.stamp = rospy.Time.now()
    pub.publish(msg)

    global first_time
    if first_time:
        pub_init.publish(initial_pose)
        first_time = False

if __name__ == '__main__':

    global first_time
    first_time = True
    rospy.init_node('lidar_timestamp_updater')

    pub_init = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=2)

    # Start point for w16 map and Jose's bag from Brazilia
    initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.stamp = rospy.Time.now()
    initial_pose.header.frame_id = "/map"
    initial_pose.pose.pose.position.x = 133.7
    initial_pose.pose.pose.position.y = 5.7
    initial_pose.pose.pose.position.z = -2.9
    initial_pose.pose.pose.orientation.w = 0.5464
    initial_pose.pose.pose.orientation.z = -0.8369

    # Start point for w16 map and demo-t2.bag
    """     initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.stamp = rospy.Time.now()
    initial_pose.header.frame_id = "/map"
    initial_pose.pose.pose.position.x = 0
    initial_pose.pose.pose.position.y = 0
    initial_pose.pose.pose.orientation.w = 1
    initial_pose.pose.pose.orientation.z = 0 """

    # Start point for w16 map and demo-2.bag from 237s (backwards)
    """     initial_pose = PoseWithCovarianceStamped()
    initial_pose.header.stamp = rospy.Time.now()
    initial_pose.header.frame_id = "/map"
    initial_pose.pose.pose.position.x = 363.72
    initial_pose.pose.pose.position.y = -642.15
    initial_pose.pose.pose.position.z = -11.1
    initial_pose.pose.pose.orientation.w = 0.522
    initial_pose.pose.pose.orientation.z = 0.853 """

    # Publish the adjusted Lidar topic
    pub = rospy.Publisher('/points_raw', PointCloud2, queue_size=10)

    # Subscribe to the original Lidar topic
    sub = rospy.Subscriber('/ouster/points', PointCloud2, callback)

    rospy.spin()
