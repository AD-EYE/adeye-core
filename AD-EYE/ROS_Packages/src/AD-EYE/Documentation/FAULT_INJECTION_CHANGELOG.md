# Fault-injection change log

This file records the AD-EYE fault-injection work added or changed during the
fault-injection development. Sections marked `AD-EYE fault-injection change`
in source files identify related additions made in code that existed before
this work.

## Fault-injection package organisation

- Moved fault-related scripts to `src/fault_injections/`.
- Updated `manager.py` to import `FaultInjectionManager` from that package.
- Added CMake installation entries for the fault scripts.
- Added `Helper_Scripts/restore_executable_permissions.sh` to restore execute
  permissions after copying the repository through media that loses Unix file
  modes.

## Vehicle-command fault manager

- Updated `FaultInjectionManager.py` code style and fault logging.
- Fault logs now use timestamped files in `~/.ros/adeye/fault_logs` by default;
  the directory can be changed through `~fault_log_dir`.
- Fault logging is event-focused, rotates at 10 MiB by default, compresses
  rotated logs, and retains the newest 20 fault-log files by default.
- Added fault-debug output on `/adeye/fault_log`.
- Added real-CAN feedback support in `manager.py`: simulation uses
  `autoware_msgs/VehicleStatus`; the physical launch uses
  `/current_velocity_phy` and `/sending_angle`.

## Sensor faults

- Added `SensorFaultInjectionManager.py`.
- Added GNSS east/north bias, GNSS dropout, GNSS no-fix, LiDAR periodic scan
  dropout, and LiDAR timestamp-offset faults.
- Added menu options 16--22 in `fault_test.py`, including combined GNSS and
  LiDAR localization-input loss.
- Added `manager_simulation_sensor_faults.launch`,
  `sensor_fault_injection_adapters.launch`, and
  `manager_real_world_sensor_faults.launch`.
- Added a source-freshness watchdog that reports stale or absent GNSS/LiDAR
  inputs without confusing intentional output dropout with an input failure.

## Physical actuator-fault route

The physical route retains the GUI topic `/vehicle_commands`; it does not add
actuator topics.

- `manager.py` selects `actuator_fault_path=physical_topics` from
  `manager_real_world.launch`. The legacy `/vehicle_cmd` callback remains for
  simulation.
- `ctrl_cmd_republisher.cpp` applies steering offset, freeze, saturation, and
  oscillation before publishing `/steering_requested_phy`.
- `vehicle_controller.cpp` applies acceleration offset, freeze, saturation,
  and oscillation before publishing `/acceleration_requested_phy`.
- The physical path ignores random steering and runaway acceleration.
- `ctrl_cmd_republisher.cpp` has a documented degrees/radians switch. Degrees
  are currently active; verify the ros2can unit before a steering test.
- Added conservative configurable physical limits and a five-second wall-clock
  timeout that clears active steering/acceleration fault state.
- Documented the outstanding physical-test blockers in
  `REAL_CAR_BLOCKERS_TODO.md`. The limits, timeout, and ROS-topic observation
  are safeguards only; they do not yet constitute CAN acknowledgement or a
  complete physical safety case.

## Experiments and user interfaces

- Added `steering_saturation_experiment.py` and its terminal helper script.
- `fault_test.py` accepts integer menu choices and can be started by
  `run_fault_menu.sh` or from Foxglove by publishing a `std_msgs/String` to
  `/vehicle_commands`.
- Added `try/finally` reset handling for individual menu faults and the
  emergency-state test.

## Documentation and validation

- Added and expanded `Documentation/FAULT_INJECTION_GUIDE.md`.
- Added `REAL_WORLD_FAULT_INJECTION.md`,
  `AUTOWARE_SIMULATION_FAULT_INJECTION.md`, and
  `REAL_CAR_BLOCKERS_TODO.md`.
- Updated the sensor, simulation, real-world, and main guides after the
  focused safety review to distinguish implemented safeguards from remaining
  blockers.
- The guide covers implemented faults, startup, rosbag remapping, observability,
  resets, simulation versus physical routing, and steering-unit selection.
- Python syntax checks, launch XML parsing, and `git diff --check` were run.
- ROS C++ compilation still needs to be performed on a ROS Kinetic machine:

  ```bash
  catkin_make --pkg adeye
  ```
