#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import numpy as np
import struct

class CloudROI:
    def __init__(self):
        rospy.init_node('cloud_ROI', anonymous=True)

        self.n_beams = rospy.get_param('~n_beams', 64)  # default 64 beams for ouster lidar OS-0-64-U13

        # Calculate the range of beams to keep (middle half)
        self.start_beam = self.n_beams // 4
        self.end_beam = self.start_beam + self.n_beams // 2

        rospy.loginfo("Cloud ROI node initialized")
        rospy.loginfo("Keeping beams {} to {} (middle half of {} beams)".format(
            self.start_beam, self.end_beam-1, self.n_beams))

        rospy.Subscriber('/os_cloud_node/points', PointCloud2, self.cloud_callback)

        self.filtered_publisher = rospy.Publisher('/os_cloud_node/ROI_points', PointCloud2, queue_size=2)

        # debug parameters
        self.last_debug_time = rospy.Time.now()
        self.debug_interval = rospy.Duration(5.0)  # debug output every 5 seconds
        self.debug_mode = rospy.get_param('~debug', False)  # enable debug logging

    def cloud_callback(self, cloud_msg):
        """Process point cloud and filter to keep only the middle beams."""
        try:

            current_time = rospy.Time.now()
            if self.debug_mode and (current_time - self.last_debug_time) > self.debug_interval:
                rospy.loginfo("---------- CLOUD DEBUG INFO ----------")
                rospy.loginfo("Point cloud height: {}, width: {}".format(cloud_msg.height, cloud_msg.width))
                rospy.loginfo("Point step: {}, row step: {}".format(cloud_msg.point_step, cloud_msg.row_step))
                rospy.loginfo("Point cloud size: {} bytes".format(len(cloud_msg.data)))

                # Get field info
                fields_info = ""
                for field in cloud_msg.fields:
                    fields_info += "{} (offset: {}), ".format(field.name, field.offset)
                rospy.loginfo("Fields: {}".format(fields_info))

                self.last_debug_time = current_time

            # filtered cloud message
            filtered_cloud = PointCloud2()
            filtered_cloud.header = cloud_msg.header
            filtered_cloud.height = cloud_msg.height
            filtered_cloud.width = cloud_msg.width
            filtered_cloud.fields = cloud_msg.fields
            filtered_cloud.is_bigendian = cloud_msg.is_bigendian
            filtered_cloud.point_step = cloud_msg.point_step
            filtered_cloud.row_step = cloud_msg.row_step  #how many bytes make up all the points in a single beam
            filtered_cloud.is_dense = cloud_msg.is_dense

            if cloud_msg.height == self.n_beams and cloud_msg.height > 1:
                # this is an organized point cloud with rows corresponding to beams

                filtered_data = bytearray(len(cloud_msg.data))

                # copy only the rows (beams) we want to keep
                for beam_idx in range(cloud_msg.height):
                    row_start = beam_idx * cloud_msg.row_step
                    row_end = row_start + cloud_msg.row_step

                    if self.start_beam <= beam_idx < self.end_beam: # the beam is in ROI
                        for i in range(row_start, row_end):
                            if i < len(cloud_msg.data):
                                filtered_data[i] = cloud_msg.data[i]

                    else:
                        # if the beam is outside our ROI, set all points to invalid (NaN or max range)

                        # offset means how many bytes from the start of a point's data that field begins.
                        # example:
                        # x_offset = 0 (x coordinate starts at the beginning of each point's data)
                        # y_offset = 4 (y coordinate starts 4 bytes in)
                        # z_offset = 8 (z coordinate starts 8 bytes in)
                        x_offset = y_offset = z_offset = None
                        for field in cloud_msg.fields:
                            if field.name == 'x':
                                x_offset = field.offset
                            elif field.name == 'y':
                                y_offset = field.offset
                            elif field.name == 'z':
                                z_offset = field.offset

                        # set x, y, z to NaN for all points in this row
                        if x_offset is not None and y_offset is not None and z_offset is not None: # 0 is not None in python
                            for col_idx in range(cloud_msg.width):
                                point_idx = row_start + col_idx * cloud_msg.point_step

                                if point_idx + x_offset + 4 <= len(filtered_data):
                                    filtered_data[point_idx+x_offset:point_idx+x_offset+4] = struct.pack('f', float('nan'))
                                if point_idx + y_offset + 4 <= len(filtered_data):
                                    filtered_data[point_idx+y_offset:point_idx+y_offset+4] = struct.pack('f', float('nan'))
                                if point_idx + z_offset + 4 <= len(filtered_data):
                                    filtered_data[point_idx+z_offset:point_idx+z_offset+4] = struct.pack('f', float('nan'))

                filtered_cloud.data = bytes(filtered_data)

            # else:
            #     # Non-organized point cloud or single-row organized cloud
            #     # Filter points by checking ring/beam index if available

            #     # Find ring/beam field offset if it exists
            #     ring_offset = None
            #     for field in cloud_msg.fields:
            #         if field.name == 'ring':
            #             ring_offset = field.offset
            #             break

            #     if ring_offset is not None:
            #         # Extract points with ring/beam indices in our ROI
            #         points = pc2.read_points(cloud_msg, skip_nans=False)
            #         filtered_points = []

            #         for point in points:
            #             # Extract ring index (varies by point cloud format)
            #             # Assuming ring is stored as uint16 or uint8
            #             # This needs to be adjusted based on actual point cloud format
            #             try:
            #                 ring_idx = int(point[ring_offset//4])  # Divide by 4 because point tuple is in 32-bit elements
            #                 if self.start_beam <= ring_idx < self.end_beam:
            #                     filtered_points.append(point)
            #             except (IndexError, ValueError):
            #                 # Skip points with invalid ring indices
            #                 pass

            #         # Create a new point cloud with filtered points
            #         filtered_cloud = pc2.create_cloud(cloud_msg.header, cloud_msg.fields, filtered_points)
            #     else:
            #         # No ring field, just pass through the original cloud for now
            #         rospy.logwarn_once("No 'ring' field found in point cloud, cannot filter by beam.")
            #         filtered_cloud = cloud_msg

            # # Publish the filtered point cloud
            # self.filtered_publisher.publish(filtered_cloud)

        except Exception as e:
            rospy.logerr("Error processing point cloud: {}".format(e))

if __name__ == '__main__':
    try:
        cloud_roi = CloudROI()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass