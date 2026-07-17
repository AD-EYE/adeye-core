#!/bin/bash

echo "================================================"
echo "Launching AD-EYE Steering Saturation Experiment"
echo "================================================"

source ~/catkin_ws/devel/setup.bash

python "$(rospack find adeye)/src/fault_injections/steering_saturation_experiment.py"
