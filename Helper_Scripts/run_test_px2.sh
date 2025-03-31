#!/bin/bash

NUM_RUNS=5
LOG_BASE_NAME="voxel_leaf_2.0_25_03_20_hs_back_GPUCheck"
PX2_LOG_DIR_ON_PC="/disk2/bags/lidar_data/px2"
DEST_DIR="/disk2/logs"
ROSBAG_DIR="/disk2/bags/lidar_data"
ROSBAG_FILE="record_25_03_20hs_back.bag"
PX2_USER="adeye"
PX2_PASSWORD="adeye"
PX2_IP="192.168.1.100"  # Replace with actual PX2 IP

# Create directories on your PC
mkdir -p "${PX2_LOG_DIR_ON_PC}" || { echo "Error creating directory ${PX2_LOG_DIR_ON_PC}"; exit 1; }
mkdir -p "${DEST_DIR}" || { echo "Error creating directory ${DEST_DIR}"; exit 1; }

# Function to start monitoring on PX2
start_px2_monitoring() {
    local run=$1
    sshpass -p "$PX2_PASSWORD" ssh -t $PX2_USER@$PX2_IP << 'EOF'
        # Create monitoring directory
        MONITOR_DIR="/tmp/px2_monitoring_run$1"
        mkdir -p "${MONITOR_DIR}"

        # Start tegrastats (GPU monitoring)
        tegrastats --interval 1000 --logfile "${MONITOR_DIR}/tegrastats.log" &
        echo $! > "${MONITOR_DIR}/tegrastats.pid"

        # Start top for CPU monitoring (1 second interval, batch mode)
        top -b -d 1 > "${MONITOR_DIR}/top.log" &
        echo $! > "${MONITOR_DIR}/top.pid"

        # Record initial CPU frequency
        cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq > "${MONITOR_DIR}/cpu_freq_start.log"
EOF
}

# Function to stop monitoring and copy logs
stop_px2_monitoring() {
    local run=$1
    local target_dir=$2

    # Stop monitoring processes
    sshpass -p "$PX2_PASSWORD" ssh -t $PX2_USER@$PX2_IP << 'EOF'
        MONITOR_DIR="/tmp/px2_monitoring_run$1"

        # Stop tegrastats
        if [ -f "${MONITOR_DIR}/tegrastats.pid" ]; then
            kill $(cat "${MONITOR_DIR}/tegrastats.pid") 2>/dev/null
        fi

        # Stop top
        if [ -f "${MONITOR_DIR}/top.pid" ]; then
            kill $(cat "${MONITOR_DIR}/top.pid") 2>/dev/null
        fi

        # Record final CPU frequency
        cat /sys/devices/system/cpu/cpu*/cpufreq/scaling_cur_freq > "${MONITOR_DIR}/cpu_freq_end.log"
EOF

    # Create monitoring directory on PC
    mkdir -p "${target_dir}/monitoring"

    # Copy monitoring files individually
    sshpass -p "$PX2_PASSWORD" scp $PX2_USER@$PX2_IP:"/tmp/px2_monitoring_run${run}/tegrastats.log" "${target_dir}/monitoring/"
    sshpass -p "$PX2_PASSWORD" scp $PX2_USER@$PX2_IP:"/tmp/px2_monitoring_run${run}/top.log" "${target_dir}/monitoring/"
    sshpass -p "$PX2_PASSWORD" scp $PX2_USER@$PX2_IP:"/tmp/px2_monitoring_run${run}/cpu_freq_*.log" "${target_dir}/monitoring/"

    # Cleanup on PX2
    sshpass -p "$PX2_PASSWORD" ssh -t $PX2_USER@$PX2_IP "rm -rf /tmp/px2_monitoring_run${run}"
}

for ((run=1; run<=NUM_RUNS; run++)); do
    echo "================================"
    echo "Starting experiment run ${run}/${NUM_RUNS}"
    echo "================================"

    # Cleanup previous temporary files
    rm -f /tmp/pid1 /tmp/pid2 /tmp/rosbag_done

    # Start monitoring on PX2
    start_px2_monitoring $run

    # Launch manager_performance_testing on PX2 via SSH
    echo "Starting manager on PX2..."
    sshpass -p "$PX2_PASSWORD" ssh -t -X $PX2_USER@$PX2_IP << 'EOF'
        echo $$ > /tmp/pid1
        source /home/adeye/catkin_ws/devel/setup.bash
        roslaunch adeye manager_bags_performance_testing.launch
EOF &
    echo $! > /tmp/px2_pid
    sleep 15  # Give more time for PX2 to start

    # Launch RViz on local PC
    echo "Starting RViz on local PC..."
    gnome-terminal -- bash -c 'echo $$ > /tmp/pid2; exec roslaunch adeye my_rviz.launch'
    sleep 5

    # Play rosbag on local PC
    echo "Starting rosbag playback..."
    gnome-terminal -- bash -c "cd ${ROSBAG_DIR} && rosbag play ${ROSBAG_FILE} --topics /fix /os_cloud_node/imu /os_cloud_node/points; touch /tmp/rosbag_done"

    # Wait for main rosbag completion
    while [ ! -f /tmp/rosbag_done ]; do
        sleep 1
    done

    # Terminate the nodes
    echo "Terminating processes..."
    kill $(cat /tmp/pid2) 2>/dev/null || echo "RViz already closed"

    # Kill PX2 processes
    sshpass -p "$PX2_PASSWORD" ssh -t $PX2_USER@$PX2_IP "kill \$(cat /tmp/pid1) 2>/dev/null || echo 'manager_performance_testing already closed'"

    # Create run-specific directory on your PC
    RUN_LOG_DIR="${PX2_LOG_DIR_ON_PC}/${LOG_BASE_NAME}_run${run}"
    mkdir -p "${RUN_LOG_DIR}"

    # Stop monitoring and copy monitoring data
    stop_px2_monitoring $run "${RUN_LOG_DIR}"

    # Copy ROS logs from PX2
    echo "Copying ROS logs from PX2..."
    sshpass -p "$PX2_PASSWORD" scp -r $PX2_USER@$PX2_IP:/home/adeye/.ros/log/latest/* "${RUN_LOG_DIR}/"

    # Optional: Also copy to original destination directory
    if [ "$DEST_DIR" != "$PX2_LOG_DIR_ON_PC" ]; then
        mkdir -p "${DEST_DIR}/${LOG_BASE_NAME}_run${run}"
        cp -r "${RUN_LOG_DIR}"/* "${DEST_DIR}/${LOG_BASE_NAME}_run${run}/"
    fi

    # Short pause between runs
    sleep 10
    echo "Completed run ${run}/${NUM_RUNS}"
    echo "Logs saved to: ${RUN_LOG_DIR}"
    echo
done

# Cleanup temporary files
rm -f /tmp/pid1 /tmp/pid2 /tmp/rosbag_done /tmp/px2_pid

echo "========================================"
echo "All ${NUM_RUNS} runs completed!"
echo "Logs available in: ${PX2_LOG_DIR_ON_PC}/${LOG_BASE_NAME}_run[1-${NUM_RUNS}]"
[ "$DEST_DIR" != "$PX2_LOG_DIR_ON_PC" ] && echo "Logs also available in: ${DEST_DIR}/${LOG_BASE_NAME}_run[1-${NUM_RUNS}]"
echo "========================================"