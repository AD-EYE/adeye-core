#!/usr/bin/env python3
import rosbag
import matplotlib.pyplot as plt
from sensor_msgs.msg import NavSatFix
import numpy as np

def read_gps_from_rosbag(bag_file, topic='/fix'):
    """
    Read GPS data from a ROSBag file.

    Args:
        bag_file (str): Path to the ROSBag file
        topic (str): GPS topic name (default: '/fix')

    Returns:
        tuple: (latitudes, longitudes, timestamps) as numpy arrays
    """
    latitudes = []
    longitudes = []
    timestamps = []

    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic]):
            if msg._type == 'sensor_msgs/NavSatFix':
                latitudes.append(msg.latitude)
                longitudes.append(msg.longitude)
                timestamps.append(t.to_sec())

    return np.array(latitudes), np.array(longitudes), np.array(timestamps)

def plot_gps_trajectory(latitudes, longitudes):
    """
    Plot GPS trajectory and analyze straightness.

    Args:
        latitudes (np.array): Array of latitude values
        longitudes (np.array): Array of longitude values
    """
    # Convert to relative coordinates (meters) using approximate conversion
    lat_to_m = 111132.92 - 559.82 * np.cos(2 * np.mean(latitudes)) + 1.175 * np.cos(4 * np.mean(latitudes))
    lon_to_m = 111412.84 * np.cos(np.mean(latitudes)) - 93.5 * np.cos(3 * np.mean(latitudes))

    # Calculate relative positions from first point
    x = (longitudes - longitudes[0]) * lon_to_m
    y = (latitudes - latitudes[0]) * lat_to_m

    # Calculate deviation from straight line
    if len(x) > 1:
        # Fit a straight line to the points
        coeffs = np.polyfit(x, y, 1)
        line_fit = np.poly1d(coeffs)
        y_fit = line_fit(x)

        # Calculate perpendicular distances from the line
        distances = (coeffs[0] * x - y + coeffs[1]) / np.sqrt(coeffs[0]**2 + 1)
        max_deviation = np.max(np.abs(distances))
        avg_deviation = np.mean(np.abs(distances))

        print("Maximum deviation from straight line: {:.2f} meters".format(max_deviation))
        print("Average deviation from straight line: {:.2f} meters".format(avg_deviation))

    # Plotting
    plt.figure(figsize=(12, 6))

    # Plot trajectory
    plt.subplot(1, 2, 1)
    plt.plot(x, y, 'b-', label='GPS Trajectory')
    if len(x) > 1:
        plt.plot(x, y_fit, 'r--', label='Best Fit Line')
    plt.xlabel('East-West Position (meters)')
    plt.ylabel('North-South Position (meters)')
    plt.title('GPS Trajectory')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()

    # Plot deviation histogram
    plt.subplot(1, 2, 2)
    if len(x) > 1:
        plt.hist(distances, bins=20, color='g', alpha=0.7)
        plt.xlabel('Deviation from straight line (meters)')
        plt.ylabel('Frequency')
        plt.title('Deviation Distribution')
        plt.grid(True)

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser(description='Visualize GPS data from ROSBag')
    parser.add_argument('bag_file', help='Path to the ROSBag file')
    parser.add_argument('--topic', default='/fix', help='GPS topic name (default: /fix)')

    args = parser.parse_args()

    # Read GPS data from rosbag
    latitudes, longitudes, timestamps = read_gps_from_rosbag(args.bag_file, args.topic)

    if len(latitudes) == 0:
        print("No GPS messages found on topic {}".format(args.topic))
    else:
        print("Found {} GPS messages".format(len(latitudes)))
        print("Time duration: {:.2f} seconds".format(timestamps[-1] - timestamps[0]))

        # Plot the trajectory
        plot_gps_trajectory(latitudes, longitudes)