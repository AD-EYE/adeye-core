#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import struct
import ctypes
from std_msgs.msg import Header
import copy

class CloudROI:
    def __init__(self):
        rospy.init_node('cloud_ROI', anonymous=True)

        self.n_beams = rospy.get_param('~n_beams', 64) # default 64 beams for ouster lidar OS-0-64-U13

        # keep middle half beams
        self.start_beam = rospy.get_param('~start_beam', self.n_beams // 4)
        self.end_beam = rospy.get_param('~end_beam', self.start_beam + self.n_beams // 2)

        rospy.loginfo("Cloud ROI node initialized")
        rospy.loginfo("Keeping beams {} to {} (of {} beams)".format(
            self.start_beam, self.end_beam-1, self.n_beams))

        # Use a queue size of 1 to avoid buffering and reduce latency
        rospy.Subscriber('/os_cloud_node/points', PointCloud2, self.cloud_callback, queue_size=1)

        self.filtered_publisher = rospy.Publisher('/os_cloud_node/ROI_points', PointCloud2, queue_size=1)

        self.last_debug_time = rospy.Time.now()

        self.debug_mode = rospy.get_param('~debug', False)  # Default false


    def cloud_callback(self, cloud_msg):
        """Process point cloud and filter to keep only the middle beams using zero-copy where possible."""
        try:
            start_time = rospy.Time.now()

            roi_height = self.end_beam - self.start_beam

            if self.debug_mode:
                rospy.loginfo("Original data size: {} bytes".format(len(cloud_msg.data)))
                rospy.loginfo("Expected row size: {} bytes".format(cloud_msg.row_step))
                rospy.loginfo("Expected total size: {} bytes".format(cloud_msg.row_step * cloud_msg.height))


            # create new point cloud with reduced height
            filtered_cloud = PointCloud2()

            # copy metadata
            filtered_cloud.header = cloud_msg.header
            filtered_cloud.height = roi_height
            filtered_cloud.width = cloud_msg.width
            filtered_cloud.fields = cloud_msg.fields
            filtered_cloud.is_bigendian = cloud_msg.is_bigendian
            filtered_cloud.point_step = cloud_msg.point_step
            filtered_cloud.row_step = cloud_msg.row_step
            filtered_cloud.is_dense = cloud_msg.is_dense

            # allocate memory for the new point cloud
            new_data = bytearray(filtered_cloud.row_step * filtered_cloud.height)

            # copy only the rows in the ROI
            for i, beam_idx in enumerate(range(self.start_beam, self.end_beam)):
                start = beam_idx * filtered_cloud.row_step
                end = start + filtered_cloud.row_step

                if self.debug_mode and i < 2:  # Just print first two beams to avoid log spam
                  rospy.loginfo("Copying beam {} (index {}) from byte {} to {}".format(beam_idx, i, start, end))

                new_data[i * filtered_cloud.row_step : (i + 1) * filtered_cloud.row_step] = cloud_msg.data[start:end]


            filtered_cloud.data = bytes(new_data)

            # # Update the header frame_id to match our ROI frame
            # # This ensures it appears correctly in RViz
            # if filtered_cloud.header.frame_id == "os_sensor":
            #     filtered_cloud.header.frame_id = "os_sensor_roi"

            if self.debug_mode:
                rospy.loginfo("---------- FOV SHRINKER PERFORMANCE ----------")
                rospy.loginfo("Input point cloud:{} points".format(cloud_msg.width * cloud_msg.height))
                rospy.loginfo("Input point cloud size: {} x {}".format(cloud_msg.width, cloud_msg.height))
                rospy.loginfo("Filtered point cloud:{} points".format(filtered_cloud.width * filtered_cloud.height))
                rospy.loginfo("Filtered point cloud size:{} x {}".format(filtered_cloud.width, filtered_cloud.height))
                rospy.loginfo("Time taken: {} ms".format((rospy.Time.now() - start_time).to_sec() * 1000.0))

            self.filtered_publisher.publish(filtered_cloud)

        except Exception as e:
            rospy.logerr("Error processing point cloud: {}".format(e))

if __name__ == '__main__':
    try:
        cloud_roi = CloudROI()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass