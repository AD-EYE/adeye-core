# AD-EYE fault-injection guide

This guide describes the faults currently implemented in AD-EYE, what they
change, what to observe in Autoware, and how to run and reset them.

> **Safety boundary:** start in simulation. Run only one fault at a time until
> its reset and observability have been verified. Physical-vehicle tests require
> a closed test area, a safety driver, an independent emergency-stop path, and
> a reviewed speed limit. `Steering random`, `runaway acceleration`, and the
> combined localization-input-loss test are simulation-only tests unless a
> separate safety case explicitly approves them.

## Fault locations

Vehicle-command faults are intended to affect commands after Autoware generates
`/vehicle_cmd`:

```text
Autoware controller -> /vehicle_cmd -> FaultInjectionManager -> vehicle
```

The current implementation subscribes to and republishes on the same
`/vehicle_cmd` topic. This means an Autoware command publisher remains normal,
but the vehicle interface can potentially receive both the original and the
modified message. Before relying on a vehicle-command result, verify that the
vehicle interface consumes the manager's modified command. A fully isolated
actuator-fault test requires a dedicated input topic and a dedicated output
topic; that routing is not implemented by the current launch files.

These faults therefore test whether vehicle-status and safety functions detect
a difference between the requested command and the affected command. Treat
physical-vehicle results as invalid if the original command can bypass the
manager to the vehicle interface.

Sensor faults are applied before Autoware consumes GNSS or LiDAR data:

```text
GNSS:  /fix -> SensorFaultInjectionManager -> /gnss_pose -> localization
LiDAR: source PointCloud2 -> SensorFaultInjectionManager -> /points_raw
       -> perception and NDT localization
```

They test perception/localization behavior when its input is degraded or
missing.

## Prerequisites and startup

Build and source the ROS workspaces in this order:

```bash
source /opt/ros/kinetic/setup.bash
source ~/autoware.ai/devel/setup.bash  # Use install/setup.bash if applicable.
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

If the repository was copied through Windows, a ZIP archive, or another medium
that loses Unix permissions, run once from the repository root:

```bash
bash Helper_Scripts/restore_executable_permissions.sh
```

### Simulation

Start the sensor-fault simulation wrapper instead of the ordinary manager
simulation launch file:

```bash
roslaunch adeye manager_simulation_sensor_faults.launch
```

It routes the simulated point cloud through `SensorFaultInjectionManager` and
configures it to publish the faulted result on `/points_raw`. Any separately
started rosbag player must also be routed through the manager; see the next
section.

### Rosbag playback with LiDAR faults

A rosbag player can publish its recorded topics directly. If it publishes
`/points_raw` directly, it bypasses `SensorFaultInjectionManager`. Autoware
then receives a mix of original and faulted scans, so LiDAR-fault results are
not valid.

Stop any current bag playback, then replay the bag with the recorded LiDAR
topic remapped to the fault-manager input:

```bash
rosbag play --clock --loop --rate 1.0 YOUR_RECORDING.bag \
  /points_raw:=/adeye/fault_injection/points_unmodified
```

Use the actual recorded LiDAR topic in place of `/points_raw` if the bag uses
a different topic name. Do not run another unremapped bag player at the same
time. `--loop` keeps the input available for repeated experiments. `--clock`
is required when `/use_sim_time` is true and rosbag is the intended time
source. In that case, `/clock` must have no other publisher. If an active
simulator already publishes an advancing `/clock`, omit `--clock` from the
rosbag command instead.

Verify that simulated time advances before starting `fault_test.py`; otherwise
its `rospy.sleep()` calls wait indefinitely:

```bash
rostopic hz /clock
```

Before selecting menu option 20 or 21, verify the route:

```bash
rostopic info /adeye/fault_injection/points_unmodified
rostopic info /points_raw
```

The bag player (for example, `/play_...`) must publish only
`/adeye/fault_injection/points_unmodified`. `/points_raw` must have exactly
one publisher:

```text
/sensor_fault_injection_manager
```

For option 20, measure a short rolling window rather than the full history:

```bash
rostopic hz -w 40 /adeye/fault_injection/points_unmodified
rostopic hz -w 40 /points_raw
```

The menu applies the fault for 10 seconds and then resets it. A large/default
measurement window averages normal scans before and after that short test with
faulted scans, hiding the real rate reduction. `-w 40` limits the calculation
to the most recent 40 messages, so it reflects the active fault after the
window fills. With an approximately 10 Hz input, `n=5` should make
`/points_raw` approximately 8 Hz and `n=2` approximately 5 Hz, with recurring
gaps of roughly 0.2 seconds. The unmodified topic remains approximately 10 Hz
in both cases. After the automatic reset, `/points_raw` returns to its
baseline rate.

GNSS faults require the simulator to publish `sensor_msgs/NavSatFix` on `/fix`.
Confirm this before enabling the GNSS broadcaster:

```bash
rostopic info /fix
```

If that topic has the required type and the GNSS map parameters are configured:

```bash
roslaunch adeye manager_simulation_sensor_faults.launch \
  enable_gnss_broadcaster:=true
```

### Physical vehicle

The physical sensor drivers are started separately. Use
`sensor_fault_injection_adapters.launch` only in place of the normal GNSS
broadcaster and normal `/points_raw` relay:

```bash
roslaunch adeye sensor_fault_injection_adapters.launch
```

Do **not** run another publisher for `/gnss_pose` or `/points_raw` at the same
time. In particular, the `points_raw_relay` in `manager_real_world.launch`
must be replaced/disabled by the approved physical-test launch configuration.
The same one-publisher rule applies to a real LiDAR driver: it must publish to
the fault-manager input, while only the manager publishes the final
`/points_raw` consumed by Autoware.

### Terminal menu

In a second terminal, source the same environments and run:

```bash
rosrun adeye fault_test.py
```

The equivalent helper script is:

```bash
bash src/fault_injections/run_fault_menu.sh
```

The menu sends `std_msgs/String` commands on `/vehicle_commands`. Foxglove can
send the same commands using a Publish panel with topic `/vehicle_commands` and
message type `std_msgs/String`, for example:

```json
{"data":"LIDAR_DROP_EVERY_N_command=5"}
```

### Pre-test routing checklist

Run these checks after starting the launch files and before beginning a test:

```bash
rosnode list | grep -E 'manager|sensor_fault_injection_manager'
rostopic info /vehicle_commands
```

For menu options 1--15, `/manager` must be running and subscribed to
`/vehicle_commands`. For options 13--15, `/manager` must also be subscribed to
`/state_cmd` where applicable. Confirm `/vehicle_cmd` publishers and
subscribers with:

```bash
rostopic info /vehicle_cmd
```

For an actuator-fault test, determine from the vehicle-interface configuration
whether it receives only the manager's modified command. If this cannot be
demonstrated, restrict options 2--12 and 15 to simulation or change the launch
routing before a physical test.

For options 16--19, confirm that `/fix` has a source and that the GNSS fault
path is complete:

```bash
rostopic info /fix
rostopic info /adeye/fault_injection/fix
rostopic info /gnss_pose
```

`/gnss_pose` must have one publisher, the `gnss_broadcaster` configured to use
`/adeye/fault_injection/fix`; a broadcaster that consumes raw `/fix` bypasses
GNSS faults.

For options 20--21, use the rosbag-routing verification in
[Rosbag playback with LiDAR faults](#rosbag-playback-with-lidar-faults).
`/points_raw` must have exactly one publisher,
`/sensor_fault_injection_manager`. Option 22 is valid only when both the GNSS
and LiDAR checks pass.

## Vehicle-command faults

Values use the units of the active Autoware vehicle interface. Confirm those
units from `/vehicle_cmd` before selecting any non-zero steering or acceleration
value. The menu defaults are legacy values, not universally safe physical-car
limits.

| Menu | Fault and command | What it changes | What it demonstrates / observe | Reset |
| --- | --- | --- | --- | --- |
| 1 | Hazard lights: `HL_command=1` | Sets both lamp commands on the affected `VehicleCmd`. | HMI/indicator path and fault logging. This is not a steering or braking fault. | `HL_command=0` |
| 2 | Wheel lock: `WLOCK_command=0` | When measured speed is below `0.1`, holds steering at the captured value and sets linear velocity to zero. | Low-speed loss of steering/drive authority and recovery. Do not request while moving. | `WLOCK_command=1` |
| 3 | Steering offset: `STEEROFFSET_command=<value>` | Adds a constant value to each requested steering angle. | Path-tracking error caused by steering bias. Observe `/vehicle_cmd`, vehicle path, and tracking error. | `STEEROFFSET_command=0` |
| 4 | Steering freeze: `STEERFREEZE_command=1` | Captures the first steering value after activation and reuses it. | Stuck steering actuator during changing curvature. Observe command/actual steering divergence. | `STEERFREEZE_command=0` |
| 5 | Steering saturation: `STEERSAT_command=<limit>` | Clips steering to the range `[-limit, +limit]`. | Loss of steering authority during a curve. Observe path deviation and recovery after reset. | `STEERSAT_command=0` |
| 6 | Steering oscillation: `STEEROSC_command=<amplitude>` | Adds a sine-wave steering disturbance with a fixed internal frequency of `2.0`. | Controller/tracker robustness to periodic steering error. | `STEEROSC_command=0` |
| 7 | Steering random: `STEERRANDOM_command=1` | Adds a new random steering value in the implementation range `[-20, +20]` to every command. | Extreme stochastic actuator disturbance. **Simulation only.** | `STEERRANDOM_command=0` |
| 8 | Acceleration offset: `ACCELOFFSET_command=<value>` | Adds a constant value to both acceleration command fields. | Longitudinal-control bias: the vehicle accelerates or decelerates differently from the request. | `ACCELOFFSET_command=0` |
| 9 | Acceleration freeze: `ACCELFREEZE_command=1` | Captures the first acceleration after activation and reuses it. | Stuck throttle/brake command behavior. | `ACCELFREEZE_command=0` |
| 10 | Acceleration saturation: `ACCELSAT_command=<limit>` | Caps only acceleration values above `limit`; it does not cap negative deceleration. | Loss of positive acceleration authority, such as reduced throttle response. | `ACCELSAT_command=0` |
| 11 | Acceleration oscillation: `ACCELOSC_command=<amplitude>` | Adds a sine-wave disturbance to both acceleration command fields. | Longitudinal controller robustness to periodic actuator error. | `ACCELOSC_command=0` |
| 12 | Runaway acceleration: `ACCELRUNAWAY_command=1` | Forces both acceleration command fields to `10.0`. | Safety response to an extreme longitudinal actuator fault. **Simulation only.** | `ACCELRUNAWAY_command=0` |
| 13 | Emergency state: `/state_cmd = emergency` | Changes the manager state to `FAULT_STATE`; it does not directly change a sensor message. | State-machine emergency transition and recovery procedure. | The menu sends `return_to_ready` after five seconds. |
| 14 | Reset all faults | Sends all vehicle and sensor reset commands. | Return to the baseline configuration after any experiment. | This is the reset action. |
| 15 | Run all tests | Runs the legacy vehicle-fault sequence, including runaway acceleration. | Regression/demo sequence. **Simulation only.** | Individual test resets are sent; run menu option 14 afterwards as a final check. |

### Standalone bounded steering-saturation experiment

`steering_saturation_experiment.py` is a more constrained terminal experiment
than menu option 5. It asks for a steering limit, a speed limit, and duration;
requires typing `ARM`; refuses a configured maximum speed above `1.0 m/s`; and
resets steering saturation in a `finally` block.

Run it with:

```bash
rosrun adeye steering_saturation_experiment.py
```

Use it for a gentle low-speed curved path, first in simulation. Observe the
requested/affected steering command, vehicle path, and recovery after reset.

## Sensor faults

Sensor commands take effect only while `SensorFaultInjectionManager` is active
and correctly routed into the Autoware data path.

| Menu | Fault and command | What it changes | What it demonstrates / observe | Reset |
| --- | --- | --- | --- | --- |
| 16 | GNSS east bias: `GNSS_EAST_BIAS_M_command=<meters>` | Changes longitude to create a local east/west GNSS offset. | Sensitivity of GNSS-assisted localization and initialization to a fixed position bias. Observe `/gnss_pose`, NDT convergence, and `/current_pose`. | `GNSS_EAST_BIAS_M_command=0` |
| 17 | GNSS north bias: `GNSS_NORTH_BIAS_M_command=<meters>` | Changes latitude to create a local north/south GNSS offset. | Same as east bias, in the north/south direction. | `GNSS_NORTH_BIAS_M_command=0` |
| 18 | GNSS dropout: `GNSS_DROPOUT_command=1` | Stops forwarding every GNSS fix. | GNSS-loss handling. LiDAR/IMU localization may continue, depending on configuration. Observe freshness of `/gnss_pose` and localization status. | `GNSS_DROPOUT_command=0` |
| 19 | GNSS no-fix: `GNSS_NO_FIX_command=1` | Forwards the fix but sets its status to `STATUS_NO_FIX`. | Whether consumers respect receiver validity/status rather than only coordinates. | `GNSS_NO_FIX_command=0` |
| 20 | LiDAR periodic scan dropout: `LIDAR_DROP_EVERY_N_command=<n>` | Drops every `n`th PointCloud2 scan. `n=5` drops 20% of scans; `n=1` drops all scans; `0` disables it. | Tolerance to reduced LiDAR frequency. Compare `rostopic hz -w 40 /adeye/fault_injection/points_unmodified` with `rostopic hz -w 40 /points_raw`; observe NDT/perception updates. | `LIDAR_DROP_EVERY_N_command=0` |
| 21 | LiDAR timestamp offset: `LIDAR_TIMESTAMP_OFFSET_S_command=<seconds>` | Offsets each LiDAR header timestamp; point coordinates/data are unchanged. | Time-synchronization robustness. Observe timestamp warnings, localization consistency, and fusion behavior. | `LIDAR_TIMESTAMP_OFFSET_S_command=0` |
| 22 | Localization input loss | Sends `GNSS_DROPOUT_command=1` and `LIDAR_DROP_EVERY_N_command=1` together. | Loss of both GNSS and LiDAR input. It is an input-loss test, not a direct suppression of `/ndt_pose` or `/current_pose`. Observe stale/degraded localization and the configured safety response. **Simulation only.** | The menu automatically sends both resets after its duration. |

## How to observe and verify faults

Use three separate checks. A command appearing on `/vehicle_commands` proves
only that the request was sent. A changed output topic or command proves the
fault was injected. A warning, fallback, safe-state transition, or controlled
recovery in Autoware proves the system response. Record all three when
possible.

### Command delivery and vehicle-command faults (options 1--15)

Confirm that the requested command is present:

```bash
rostopic echo /vehicle_commands
```

For vehicle-command faults, inspect the manager's fault log and the affected
command/status topics:

```bash
rostopic echo /adeye/fault_log
rostopic echo /vehicle_cmd
rostopic echo /vehicle_status
```

`/adeye/fault_log` is emitted by `FaultInjectionManager` when it modifies a
vehicle command. For example, saturation should report a steering-saturation
event while the requested steering exceeds the limit. Verify the affected
command field, the vehicle status, path tracking, and the safety response in
Foxglove or RViz. Option 13 should also cause a change on `manager/state` and
may trigger `/safety_channel/switch_request`.

Because the current vehicle-command path shares `/vehicle_cmd` between the
original and republished messages, a changed message on this topic alone does
not prove that the vehicle interface used the modified one. Confirm the
vehicle-interface routing before treating an actuator-fault result as valid,
especially on a physical vehicle.

### GNSS faults (options 16--19)

Observe the raw and faulted GNSS streams side by side:

```bash
rostopic echo -n 1 /fix
rostopic echo -n 1 /adeye/fault_injection/fix
rostopic hz -w 20 /fix
rostopic hz -w 20 /adeye/fault_injection/fix
```

For options 16 and 17, latitude or longitude on the faulted topic must differ
from `/fix` by the requested local bias. For option 18, `/fix` continues but
the faulted topic stops. For option 19, the faulted message's `status.status`
must be `STATUS_NO_FIX`. Then inspect `/gnss_pose`, `/ndt_pose`, and
`/current_pose` for the configured localization response.

### LiDAR faults (options 20--21)

For option 20, use the rolling-window commands documented in
[Rosbag playback with LiDAR faults](#rosbag-playback-with-lidar-faults).
The unmodified stream must retain its baseline rate. The final `/points_raw`
rate must change only while the fault is active: a 10 Hz input becomes about
8 Hz for `n=5`, about 5 Hz for `n=2`, and produces no output for `n=1`.

For option 21, compare the headers directly:

```bash
rostopic echo -n 1 /adeye/fault_injection/points_unmodified
rostopic echo -n 1 /points_raw
```

The PointCloud2 payload is intentionally unchanged. The observable injected
effect is the configured difference between the two `header.stamp` values.
Also inspect Autoware logs and NDT/perception behavior for time-synchronization
warnings or degraded updates.

### Localization input loss (option 22)

Option 22 is verified only when both source streams remain active while both
faulted streams stop:

```bash
rostopic hz -w 20 /fix
rostopic hz -w 20 /adeye/fault_injection/fix
rostopic hz -w 40 /adeye/fault_injection/points_unmodified
rostopic hz -w 40 /points_raw
```

During the test, the raw GNSS and LiDAR rates should remain non-zero, while the
two faulted outputs should produce no new messages. Observe the age/freshness
and behavior of `/gnss_pose`, `/ndt_pose`, and `/current_pose`; these pose
topics may continue briefly because downstream nodes can retain their last
valid estimate. A stale pose is not the same as a confirmed safe response—also
record the configured warning, fallback, or safety action.

## Manual commands and reset

Send one command manually from a sourced terminal:

```bash
rostopic pub -1 /vehicle_commands std_msgs/String \
  "data: 'STEERSAT_command=0.10'"
```

Reset all sensor faults manually:

```bash
rostopic pub -1 /vehicle_commands std_msgs/String "data: 'GNSS_EAST_BIAS_M_command=0'"
rostopic pub -1 /vehicle_commands std_msgs/String "data: 'GNSS_NORTH_BIAS_M_command=0'"
rostopic pub -1 /vehicle_commands std_msgs/String "data: 'GNSS_DROPOUT_command=0'"
rostopic pub -1 /vehicle_commands std_msgs/String "data: 'GNSS_NO_FIX_command=0'"
rostopic pub -1 /vehicle_commands std_msgs/String "data: 'LIDAR_DROP_EVERY_N_command=0'"
rostopic pub -1 /vehicle_commands std_msgs/String "data: 'LIDAR_TIMESTAMP_OFFSET_S_command=0'"
```

For a complete reset, use terminal-menu option 14. It resets the vehicle and
sensor faults together.

## Recommended observations

Before a test, record a baseline. During and after every test, inspect or
record at least:

```text
/vehicle_cmd
/vehicle_status
/points_raw
/gnss_pose
/ndt_pose
/current_pose
/adeye/fault_log
```

Useful checks:

```bash
rostopic hz -w 40 /points_raw
rostopic echo -n 1 /gnss_pose
rostopic echo -n 1 /ndt_pose
```

Fault logs are written by `FaultInjectionManager` under
`~/.ros/adeye/fault_logs` by default. The directory can be overridden with the
private ROS parameter `~fault_log_dir`.

## Interpretation

A changed input or command alone is not proof that Autoware handled the fault
correctly. A successful test has a defined expected outcome: for example,
degraded-localization detection, a warning, a fallback behavior, or a safe
stop. Record both the injected fault and the observed system response, then
reset and verify that normal operation returns.
