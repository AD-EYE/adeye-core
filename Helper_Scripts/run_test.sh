#!/bin/bash

NUM_RUNS=5
LOG_BASE_NAME="voxel_leaf_2.0_25_13_20_hs_back_GPUCheck"
DEST_DIR="/disk2/logs"
ROSBAG_DIR="/disk2/bags/lidar_data"
ROSBAG_FILE="record_25_03_20_hs_back.bag"

# Create destination directory
mkdir -p "${DEST_DIR}" || { echo "Error creating directory ${DEST_DIR}"; exit 1; }

for ((run=1; run<=NUM_RUNS; run++)); do
    echo "================================"
    echo "Starting experiment run ${run}/${NUM_RUNS}"
    echo "================================"

    # Cleanup previous temporary files
    rm -f /tmp/pid1 /tmp/pid2 /tmp/rosbag_done

    # Launch manager_performance_testing
    gnome-terminal -- bash -c 'echo $$ > /tmp/pid1; exec roslaunch adeye manager_performance_testing.launch'
    sleep 10

    # Launch RViz
    gnome-terminal -- bash -c 'echo $$ > /tmp/pid2; exec roslaunch adeye my_rviz.launch'
    sleep 5

    #gnome-terminal -- bash -c "cd ${ROSBAG_DIR} && rosbag play ${ROSBAG_FILE} --topics /fix --duration=15"

    # Play other topics until end of bag
    gnome-terminal -- bash -c "cd ${ROSBAG_DIR} && rosbag play ${ROSBAG_FILE} --topics /fix /os_cloud_node/imu /os_cloud_node/points; touch /tmp/rosbag_done"

    # Wait for main rosbag completion
    while [ ! -f /tmp/rosbag_done ]; do
        sleep 1
    done

    # Terminate the nodes
    kill $(cat /tmp/pid1) 2>/dev/null || echo "manager_performance_testing already closed"
    kill $(cat /tmp/pid2) 2>/dev/null || echo "RViz already closed"

    # Copy and rename log directory
    run_log_dir="${DEST_DIR}/${LOG_BASE_NAME}_run${run}"
    cp -r "/home/adeye/.ros/log/latest" "${run_log_dir}" || echo "Error copying logs for run ${run}"

    # Short pause between runs
    sleep 10

    echo "Completed run ${run}/${NUM_RUNS}"
    echo "Logs saved to: ${run_log_dir}"
    echo
done

# Cleanup temporary files
rm -f /tmp/pid1 /tmp/pid2 /tmp/rosbag_done

echo "========================================"
echo "All ${NUM_RUNS} runs completed!"
echo "Logs available in: ${DEST_DIR}/${LOG_BASE_NAME}_run[1-${NUM_RUNS}]"
echo "========================================"
