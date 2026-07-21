# Sensor fault injection

`SensorFaultInjectionManager.py` injects faults into GNSS and LiDAR data before
Autoware consumes them. It listens for commands on `/vehicle_commands` and does
not modify `FaultInjectionManager.py`, `gnss_broadcaster.py`, or
`point_cloud_broadcaster.py`.

It reports a stale or absent raw GNSS/LiDAR input after
`~sensor_input_timeout_s` seconds (default: two seconds). This watchdog only
logs the condition; it does not request a manager safe state or prove that
Autoware responded safely.

## Commands

| Command | Effect |
| --- | --- |
| `GNSS_EAST_BIAS_M_command=<meters>` | Offsets longitude east/west. |
| `GNSS_NORTH_BIAS_M_command=<meters>` | Offsets latitude north/south. |
| `GNSS_DROPOUT_command=1` | Drops all GNSS fixes. |
| `GNSS_DROPOUT_command=0` | Resumes GNSS fixes. |
| `GNSS_NO_FIX_command=1` | Marks fixes with `STATUS_NO_FIX`. |
| `GNSS_NO_FIX_command=0` | Preserves the receiver status. |
| `LIDAR_DROP_EVERY_N_command=<n>` | Drops every nth LiDAR scan; `0` disables it. |
| `LIDAR_TIMESTAMP_OFFSET_S_command=<seconds>` | Offsets every LiDAR timestamp; `0` disables it. |

## Launching

Use `sensor_fault_injection_adapters.launch` in place of the normal GNSS
broadcaster and the normal `/points_raw` relay. The physical LiDAR driver and
GNSS receiver must still be launched separately.

```bash
roslaunch adeye sensor_fault_injection_adapters.launch
```

For the complete physical manager setup, prefer:

```bash
roslaunch adeye manager_real_world_sensor_faults.launch
```

The launch file creates this route:

```text
/fix -> sensor fault injector -> /adeye/fault_injection/fix
     -> existing gnss_broadcaster -> /gnss_pose

/os_cloud_node/points -> sensor fault injector
                       -> /adeye/fault_injection/points
                       -> /points_raw -> Autoware
```

Do not simultaneously run another publisher for `/gnss_pose` or `/points_raw`.
For the existing real-world manager launch, this means its `points_raw_relay`
must not also be active. This launch file intentionally does not start the
Ouster driver or GNSS receiver, so it can be used with their existing launches.

### Simulation

Use the simulation wrapper instead of `manager_simulation.launch` when testing
LiDAR faults:

```bash
roslaunch adeye manager_simulation_sensor_faults.launch
```

It redirects the existing simulated `PointCloud2` stream through the injector
and restores it as the sole `/points_raw` publisher. GNSS injection requires
the simulator to publish `sensor_msgs/NavSatFix` on `/fix`; enable the existing
GNSS broadcaster only after confirming that source and its map parameters:

```bash
roslaunch adeye manager_simulation_sensor_faults.launch \
  enable_gnss_broadcaster:=true
```

## Example commands

```bash
rostopic pub -1 /vehicle_commands std_msgs/String "data: 'GNSS_EAST_BIAS_M_command=2.0'"
rostopic pub -1 /vehicle_commands std_msgs/String "data: 'LIDAR_DROP_EVERY_N_command=5'"
```

## Foxglove controls

The sensor injector subscribes directly to `/vehicle_commands`, so no adapter
node is needed for Foxglove. Add a Publish panel with message type
`std_msgs/String`, topic `/vehicle_commands`, and publish one command at a
time. For example:

```json
{"data":"GNSS_DROPOUT_command=1"}
```

Use separate buttons for each reset command. A localization-input-loss test is
the pair `GNSS_DROPOUT_command=1` and `LIDAR_DROP_EVERY_N_command=1`; reset
both commands when the experiment ends.

## Safety limits and physical use

GNSS bias and LiDAR timestamp-offset values currently accept any finite
number. Treat them as simulation-first faults: define reviewed limits before a
physical test, record the expected Autoware response, and complete
`REAL_CAR_BLOCKERS_TODO.md`. Do not assume that a watchdog log is a safety
response.

Reset after every experiment:

```bash
rostopic pub -1 /vehicle_commands std_msgs/String "data: 'GNSS_EAST_BIAS_M_command=0'"
rostopic pub -1 /vehicle_commands std_msgs/String "data: 'GNSS_NORTH_BIAS_M_command=0'"
rostopic pub -1 /vehicle_commands std_msgs/String "data: 'GNSS_DROPOUT_command=0'"
rostopic pub -1 /vehicle_commands std_msgs/String "data: 'GNSS_NO_FIX_command=0'"
rostopic pub -1 /vehicle_commands std_msgs/String "data: 'LIDAR_DROP_EVERY_N_command=0'"
rostopic pub -1 /vehicle_commands std_msgs/String "data: 'LIDAR_TIMESTAMP_OFFSET_S_command=0'"
```
