# Autoware simulation fault-injection procedure

Use this procedure before any physical-vehicle test.

> Do not use `manager_real_world.launch` or
> `actuator_fault_path:=physical_topics` for Autoware simulation tests.

## Start the fault route

```bash
source /opt/ros/kinetic/setup.bash
source ~/autoware.ai/devel/setup.bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash

roslaunch adeye manager_simulation_sensor_faults.launch
```

The simulation launch retains the legacy `/vehicle_cmd` fault route and sends
LiDAR through `SensorFaultInjectionManager` before `/points_raw`.

`/vehicle_cmd` is a simulation-only fault route. Do not transfer that launch
parameter or routing design to the physical car.

## LiDAR rosbag playback

Do not publish a bag directly to `/points_raw`, because that bypasses the
sensor fault manager. Use:

```bash
rosbag play --clock --loop --rate 1.0 YOUR_RECORDING.bag \
  /points_raw:=/adeye/fault_injection/points_unmodified
```

If another simulator owns an advancing `/clock`, omit `--clock`. Confirm that
only the sensor fault manager publishes `/points_raw`:

```bash
rostopic info /adeye/fault_injection/points_unmodified
rostopic info /points_raw
```

## Run the menu

```bash
rosrun adeye fault_test.py
```

Foxglove can publish the same `std_msgs/String` commands on
`/vehicle_commands`.

## Observe faults

For LiDAR periodic dropout, use a rolling window because menu faults are
short-lived:

```bash
rostopic hz -w 40 /adeye/fault_injection/points_unmodified
rostopic hz -w 40 /points_raw
```

For GNSS faults:

```bash
rostopic echo -n 1 /fix
rostopic echo -n 1 /adeye/fault_injection/fix
```

For vehicle-command faults, inspect `/adeye/fault_log`, `/vehicle_cmd`,
`/vehicle_status`, and the Autoware path/localization response. The legacy
simulation `/vehicle_cmd` route is not proof of a real-car actuator result.

## Reset

Use menu option 14 after every trial. `fault_test.py` now uses `try/finally`
to send an individual fault reset even if a test is interrupted.

The sensor fault injector logs an error if either input has gone stale for
longer than its configured `~sensor_input_timeout_s` (default: two seconds).
Intentional GNSS/LiDAR dropout faults do not trigger this watchdog because the
input remains active; only forwarding is suppressed.

Vehicle-fault values in the legacy simulation manager are not physically
bounded and its parser does not yet reject `NaN` or `inf`. Use finite,
scenario-reviewed values even in simulation, and do not treat a successful
simulation test as proof of physical actuator safety.
