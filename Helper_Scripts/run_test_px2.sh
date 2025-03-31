#!/bin/bash

NUM_RUNS=5

LOG_BASE_NAME="voxel_leaf_2.0_25_03_20_hs_back_GPUCheck"

DEST_DIR="/disk2/logs"

ROSBAG_DIR="/disk2/bags/lidar_data"

ROSBAG_FILE="record_25_03_20hs_back.bag"

PX2_USER="adeye"
PX2_HOST="px2.local"
PX2_PASSWORD="adeye"
PX2_LOG_DIR="/home/adeye/.ros/log/latest"
PX2_CPU_LOG="/tmp/cpu_usage.log"
PX2_GPU_LOG="/tmp/gpu_usage.log"
LOCAL_PX2_LOG_DIR="${ROSBAG_DIR}/px2"

# Create destination directories
mkdir -p "${DEST_DIR}" || { echo "Error creating directory ${DEST_DIR}"; exit 1; }
mkdir -p "${LOCAL_PX2_LOG_DIR}" || { echo "Error creating directory ${LOCAL_PX2_LOG_DIR}"; exit 1; }

for ((run=1; run<=NUM_RUNS; run++)); do
    echo "================================"
    echo "Starting experiment run ${run}/${NUM_RUNS}"
    echo "================================"

    # Cleanup previous temporary files
    rm -f /tmp/pid1 /tmp/pid2 /tmp/rosbag_done

    # Launch manager_performance_testing on PX2
    sshpass -p "${PX2_PASSWORD}" ssh -o StrictHostKeyChecking=no ${PX2_USER}@${PX2_HOST} "gnome-terminal -- bash -c 'echo $$ > /tmp/pid1; exec roslaunch adeye manager_bags_performance_testing.launch'"

    sleep 10

    # Start CPU and GPU monitoring on PX2
    sshpass -p "${PX2_PASSWORD}" ssh ${PX2_USER}@${PX2_HOST} "top -b -d 1 -c -w 10000 -o '%CPU' > ${PX2_CPU_LOG} & tagrastats > ${PX2_GPU_LOG} & echo $! > /tmp/monitor_pid"

    # Launch RViz on local machine
    gnome-terminal -- bash -c 'echo $$ > /tmp/pid2; exec roslaunch adeye my_rviz.launch'

    sleep 5

    # Play rosbag on local machine
    gnome-terminal -- bash -c "cd ${ROSBAG_DIR} && rosbag play ${ROSBAG_FILE} --topics /fix /os_cloud_node/imu /os_cloud_node/points; touch /tmp/rosbag_done"

    # Wait for main rosbag completion
    while [ ! -f /tmp/rosbag_done ]; do
        sleep 1
    done

    # Terminate local nodes
    kill $(cat /tmp/pid2) 2>/dev/null || echo "RViz already closed"

    # Terminate process on PX2
    sshpass -p "${PX2_PASSWORD}" ssh ${PX2_USER}@${PX2_HOST} "kill \\$(cat /tmp/pid1) 2>/dev/null || echo 'manager_performance_testing already closed'"

    # Stop monitoring on PX2
    sshpass -p "${PX2_PASSWORD}" ssh ${PX2_USER}@${PX2_HOST} "kill \\$(cat /tmp/monitor_pid) 2>/dev/null || echo 'Monitoring already stopped'"

    # Copy logs from PX2 to local machine
    run_log_dir="${LOCAL_PX2_LOG_DIR}/${LOG_BASE_NAME}_run${run}"
    mkdir -p "${run_log_dir}"
    sshpass -p "${PX2_PASSWORD}" scp -r ${PX2_USER}@${PX2_HOST}:${PX2_LOG_DIR} "${run_log_dir}" || echo "Error copying logs for run ${run}"
    sshpass -p "${PX2_PASSWORD}" scp ${PX2_USER}@${PX2_HOST}:${PX2_CPU_LOG} "${run_log_dir}/cpu_usage_run${run}.log" || echo "Error copying CPU log for run ${run}"
    sshpass -p "${PX2_PASSWORD}" scp ${PX2_USER}@${PX2_HOST}:${PX2_GPU_LOG} "${run_log_dir}/gpu_usage_run${run}.log" || echo "Error copying GPU log for run ${run}"

    sleep 10

    echo "Completed run ${run}/${NUM_RUNS}"
    echo "Logs saved to: ${run_log_dir}"
    echo

done

# Cleanup temporary files
rm -f /tmp/pid1 /tmp/pid2 /tmp/rosbag_done

echo "========================================"
echo "All ${NUM_RUNS} runs completed!"
echo "Logs available in: ${LOCAL_PX2_LOG_DIR}/${LOG_BASE_NAME}_run[1-${NUM_RUNS}]"
echo "========================================"