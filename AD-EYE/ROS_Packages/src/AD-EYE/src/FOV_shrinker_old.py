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
        self.debug_interval = rospy.Duration(5.0)  # Debug output every 5 seconds
        self.debug_mode = rospy.get_param('~debug', False)  # Default false

        self.processing_times = []
        self.max_times_to_track = 100

    def cloud_callback(self, cloud_msg):
        """Process point cloud and filter to keep only the middle beams using zero-copy where possible."""
        try:
            start_time = rospy.Time.now()

            # For organized point cloud ,height = number of beams, width = azimuth(horizontal measurement)
            if cloud_msg.height == self.n_beams and cloud_msg.height > 1:
                # Create a shallow copy of the message to preserve most metadata
                # This avoids copying the entire data array initially
                filtered_cloud = copy.copy(cloud_msg)

                # Find x, y, z field offsets
                x_offset = y_offset = z_offset = None
                for field in cloud_msg.fields:
                    if field.name == 'x':
                        x_offset = field.offset
                    elif field.name == 'y':
                        y_offset = field.offset
                    elif field.name == 'z':
                        z_offset = field.offset


                data_array = bytearray(cloud_msg.data)

                # Only modify the rows outside our ROI - direct memory modification
                for beam_idx in range(cloud_msg.height):
                    if beam_idx < self.start_beam or beam_idx >= self.end_beam:
                        # if the beam is outside our ROI, set points to NaN
                        row_start = beam_idx * cloud_msg.row_step

                        # Set only the x,y,z values to NaN for all points in this row
                        if x_offset is not None and y_offset is not None and z_offset is not None:
                            nan_bytes = struct.pack('f', float('nan'))
                            for col_idx in range(cloud_msg.width):
                                point_idx = row_start + col_idx * cloud_msg.point_step

                                # Avoid copy operations by directly setting bytes
                                if point_idx + x_offset + 4 <= len(data_array):
                                    data_array[point_idx+x_offset:point_idx+x_offset+4] = nan_bytes
                                if point_idx + y_offset + 4 <= len(data_array):
                                    data_array[point_idx+y_offset:point_idx+y_offset+4] = nan_bytes
                                if point_idx + z_offset + 4 <= len(data_array):
                                    data_array[point_idx+z_offset:point_idx+z_offset+4] = nan_bytes

                # Update the header timestamp to be more current (reduces perceived latency)
                filtered_cloud.header.stamp = rospy.Time.now()

                # Assign the modified data buffer
                filtered_cloud.data = bytes(data_array)

                # Publish the filtered cloud
                self.filtered_publisher.publish(filtered_cloud)

            # else:
            #     # Handle unorganized clouds differently
            #     # For unorganized clouds with ring field, use more efficient approach
            #     ring_offset = None
            #     for field_idx, field in enumerate(cloud_msg.fields):
            #         if field.name == 'ring':
            #             ring_offset = field.offset
            #             ring_field_idx = field_idx
            #             break

            #     if ring_offset is not None:
            #         # For unorganized clouds, we filter directly during iteration
            #         # Create a new point cloud with same metadata
            #         filtered_cloud = copy.copy(cloud_msg)
            #         data_array = bytearray(cloud_msg.data)

            #         point_step = cloud_msg.point_step
            #         x_offset = y_offset = z_offset = None
            #         for field in cloud_msg.fields:
            #             if field.name == 'x':
            #                 x_offset = field.offset
            #             elif field.name == 'y':
            #                 y_offset = field.offset
            #             elif field.name == 'z':
            #                 z_offset = field.offset

            #         # If we have all necessary offsets, process the cloud
            #         if x_offset is not None and y_offset is not None and z_offset is not None:
            #             nan_bytes = struct.pack('f', float('nan'))

            #             # Process the data buffer directly
            #             for point_idx in range(0, len(data_array), point_step):
            #                 if point_idx + ring_offset + 2 <= len(data_array):  # assuming uint16 for ring
            #                     # Extract ring index - this depends on data type of ring field
            #                     if cloud_msg.fields[ring_field_idx].datatype == PointField.UINT16:
            #                         ring_idx = struct.unpack('H', data_array[point_idx+ring_offset:point_idx+ring_offset+2])[0]
            #                     elif cloud_msg.fields[ring_field_idx].datatype == PointField.UINT8:
            #                         ring_idx = data_array[point_idx+ring_offset]
            #                     else:
            #                         # Default to uint16 if type is unexpected
            #                         ring_idx = struct.unpack('H', data_array[point_idx+ring_offset:point_idx+ring_offset+2])[0]

            #                     # If outside ROI, set to NaN
            #                     if ring_idx < self.start_beam or ring_idx >= self.end_beam:
            #                         if point_idx + x_offset + 4 <= len(data_array):
            #                             data_array[point_idx+x_offset:point_idx+x_offset+4] = nan_bytes
            #                         if point_idx + y_offset + 4 <= len(data_array):
            #                             data_array[point_idx+y_offset:point_idx+y_offset+4] = nan_bytes
            #                         if point_idx + z_offset + 4 <= len(data_array):
            #                             data_array[point_idx+z_offset:point_idx+z_offset+4] = nan_bytes

            #             # Update the header timestamp
            #             filtered_cloud.header.stamp = rospy.Time.now()
            #             filtered_cloud.data = bytes(data_array)
            #             self.filtered_publisher.publish(filtered_cloud)
            #         else:
            #             # Can't process without knowing field offsets
            #             rospy.logwarn_once("Missing x, y, or z field offsets in point cloud")
            #             self.filtered_publisher.publish(cloud_msg)  # Pass through
            #     else:
            #         # No ring field, just pass through the original cloud
            #         rospy.logwarn_once("No 'ring' field found in point cloud, cannot filter by beam")
            #         self.filtered_publisher.publish(cloud_msg)  # Pass through

            # Track processing time
            end_time = rospy.Time.now()
            processing_duration = (end_time - start_time).to_sec() * 1000.0  # Convert to milliseconds
            self.processing_times.append(processing_duration)
            if len(self.processing_times) > self.max_times_to_track:
                self.processing_times.pop(0)

            # Debug output
            current_time = rospy.Time.now()
            if self.debug_mode and (current_time - self.last_debug_time) > self.debug_interval:
                avg_time = sum(self.processing_times) / len(self.processing_times) if self.processing_times else 0
                max_time = max(self.processing_times) if self.processing_times else 0

                rospy.loginfo("---------- CLOUD ROI PERFORMANCE ----------")
                rospy.loginfo("Point cloud size: {} points".format(cloud_msg.width * cloud_msg.height))
                rospy.loginfo("Original point cloud size {} x {} ".format(cloud_msg.width, cloud_msg.height))
                rospy.loginfo("Output point cloud size {} x {}".format(filtered_cloud.width, filtered_cloud.height))
                rospy.loginfo("Processing time: {:.2f} ms (avg: {:.2f} ms, max: {:.2f} ms)".format(
                    processing_duration, avg_time, max_time))
                rospy.loginfo("Message timestamp latency: {:.2f} ms".format(
                    (rospy.Time.now() - cloud_msg.header.stamp).to_sec() * 1000.0))
                self.last_debug_time = current_time

        except Exception as e:
            rospy.logerr("Error processing point cloud: {}".format(e))

if __name__ == '__main__':
    try:
        cloud_roi = CloudROI()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass