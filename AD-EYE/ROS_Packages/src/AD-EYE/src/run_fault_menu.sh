#!/bin/bash

echo "======================================="
echo "Launching AD-EYE Fault Injection Tester"
echo "======================================="

source ~/catkin_ws/devel/setup.bash

rosrun adeye fault_test.py