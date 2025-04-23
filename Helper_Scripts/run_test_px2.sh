#!/bin/bash

NUM_RUNS=5
LOG_BASE_NAME="px2_voxel_leaf_0.2"
DEST_DIR="/disk2/logs"
ROSBAG_DIR="/disk2/bags/lidar_data"
ROSBAG_FILE="record_25_03_20_hs.bag"

PX2_USER="adeye"
PX2_HOST="tegra-a.local"  # Changed to hostname for consistency
PX2_PASSWORD="adeye"
PX2_LOG_DIR="/home/adeye/.ros/log/latest"
PX2_CPU_LOG="${ROSBAG_DIR}/px2/cpu_usage"
PX2_GPU_LOG="${ROSBAG_DIR}/px2/gpu_usage"
LOCAL_PX2_LOG_DIR="${ROSBAG_DIR}/px2"

# Create destination directories
mkdir -p "${DEST_DIR}" || { echo "Error creating directory ${DEST_DIR}"; exit 1; }
mkdir -p "${LOCAL_PX2_LOG_DIR}" || { echo "Error creating directory ${LOCAL_PX2_LOG_DIR}"; exit 1; }

for ((run=1; run<=NUM_RUNS; run++)); do
    echo "================================"
    echo "Starting experiment run ${run}/${NUM_RUNS}"
    echo "================================"

    # Cleanup previous temporary files
    rm -f /tmp/pid1 /tmp/pid2 /tmp/rosbag_done /tmp/terminal_pid /tmp/monitor_pid /tmp/cpu_monitor_pid

    # Launch manager_performance_testing on PX2 (with proper process tracking)
    gnome-terminal --disable-factory --title="PX2 Node - Run ${run}" -- bash -c "
        echo \$\$ > /tmp/terminal_pid
        sshpass -p '${PX2_PASSWORD}' ssh -t ${PX2_USER}@${PX2_HOST} '
            bash --login -c \"source /opt/ros/kinetic/setup.bash &&
            source ~/AD-EYE_Core/AD-EYE/ROS_Packages/devel/setup.bash &&
            (set -m; roslaunch adeye manager_bag_performance_testing.launch &
            echo \\\$! > /tmp/roslaunch_pid &&
            echo \\\$! > /tmp/pgid &&
            wait)\"'"

    sleep 45  # Wait for PX2 initialization

    # Launch CPU monitoring in dedicated window
    gnome-terminal --title="PX2 CPU Monitor - Run ${run}" -- bash -c "
        echo \$\$ > /tmp/cpu_monitor_pid
        sshpass -p '${PX2_PASSWORD}' ssh -t ${PX2_USER}@${PX2_HOST} '
            echo \"==== CPU Monitoring Started - Run ${run} ====\"
            echo \"Logging to: ${PX2_CPU_LOG}_run${run}.log\"
            top -b -d 1 -c -w 10000 -o \"%CPU\" | tee \"${PX2_CPU_LOG}_run${run}.log\"'
        echo \"CPU Monitoring Completed\"
        read -p \"Press Enter to close...\""

    # Launch RViz on local machine
    gnome-terminal --title="Local RViz - Run ${run}" -- bash -c 'echo $$ > /tmp/pid2; exec roslaunch adeye my_rviz.launch'
    sleep 10  # Wait for RViz initialization

    # Play rosbag on local machine (with completion marker)
    gnome-terminal --title="Rosbag Player - Run ${run}" -- bash -c "cd ${ROSBAG_DIR} && rosbag play ${ROSBAG_FILE} --topics /fix /os_cloud_node/imu /os_cloud_node/points; touch /tmp/rosbag_done"

    # Wait for rosbag completion
    while [ ! -f /tmp/rosbag_done ]; do
        sleep 1
    done

    # --- Cleanup Phase ---

    # 1. Terminate RViz
    kill $(cat /tmp/pid2) 2>/dev/null || echo "RViz already closed"
    rm -f /tmp/pid2

    # 2. Terminate CPU monitor
    if [ -f /tmp/cpu_monitor_pid ]; then
        kill $(cat /tmp/cpu_monitor_pid) 2>/dev/null || echo "CPU monitor already closed"
        rm -f /tmp/cpu_monitor_pid
    fi

    # 3. Terminate PX2 processes (safely)
    sshpass -p "${PX2_PASSWORD}" ssh ${PX2_USER}@${PX2_HOST} "
        # Send SIGINT to process group
        kill -INT -- -\$(cat /tmp/pgid 2>/dev/null) 2>/dev/null
        sleep 2
        # Force kill if still running
        kill -9 -- -\$(cat /tmp/pgid 2>/dev/null) 2>/dev/null
        # Cleanup ROS nodes
        rosnode kill -a 2>/dev/null
        rosnode cleanup 2>/dev/null
        # Cleanup PID files
        rm -f /tmp/roslaunch_pid /tmp/pgid /tmp/monitor_pid"

    # 4. Close PX2 terminal window
    if [ -f /tmp/terminal_pid ]; then
        kill $(cat /tmp/terminal_pid) 2>/dev/null
        rm -f /tmp/terminal_pid
    fi

    # Final cleanup
    rm -f /tmp/rosbag_done

    # Copy logs from PX2
    sshpass -p "${PX2_PASSWORD}" scp ${PX2_USER}@${PX2_HOST}:${PX2_LOG_DIR}/* "${LOCAL_PX2_LOG_DIR}/${LOG_BASE_NAME}_run${run}" || echo "Warning: Failed to copy logs from PX2"

    echo "Completed run ${run}/${NUM_RUNS}"
    echo "--------------------------------"
done

# Final cleanup
rm -f /tmp/pid1 /tmp/pid2 /tmp/rosbag_done /tmp/terminal_pid /tmp/monitor_pid /tmp/cpu_monitor_pid

echo "========================================"
echo "All ${NUM_RUNS} runs completed!"
echo "Logs available in: ${LOCAL_PX2_LOG_DIR}/${LOG_BASE_NAME}_run[1-${NUM_RUNS}]"
echo "CPU logs: ${PX2_CPU_LOG}_run[1-${NUM_RUNS}].log"
echo "========================================"
 # gnome-terminal -- bash -c "sshpass -p 'adeye' ssh -t adeye@tegra-a.local 'bash --login -c \"source /opt/ros/kinetic/setup.bash && source ~/AD-EYE_Core/AD-EYE/ROS_Packages/devel/setup.bash && roslaunch adeye manager_bag_performance_testing.launch\"'; exec bash" # gnome-terminal -- bash -c "sshpass -p 'adeye' ssh -t adeye@tegra-a.local 'bash --login -c \"source /opt/ros/kinetic/setup.bash && source ~/AD-EYE_Core/AD-EYE/ROS_Packages/devel/setup.bash && roslaunch adeye manager_bag_performance_testing.launch\"'; exec bash"