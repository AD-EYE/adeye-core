# Real-world fault-injection procedure

This procedure is for a stationary vehicle or a separately approved closed
test area. It is not a substitute for a vehicle safety case.

> Do not use `actuator_fault_path:=vehicle_cmd` on the real car. That is the
> legacy simulation route and republishes `/vehicle_cmd` on the same topic.
>
> The current direct actuator route is not approved for non-zero physical
> injection until every item in `REAL_CAR_BLOCKERS_TODO.md` is completed.

## Physical command route

```text
GUI -> /vehicle_commands -> direct actuator publisher -> ros2can -> CAN
```

Steering faults are applied by `ctrl_cmd_republisher` before
`/steering_requested_phy`. Acceleration faults are applied by
`vehicle_controller` before `/acceleration_requested_phy`.

## Before starting

1. Build and source the workspace.

   ```bash
   cd ~/catkin_ws
   catkin_make --pkg adeye
   source devel/setup.bash
   ```

2. Confirm the steering unit configured in `ctrl_cmd_republisher.cpp`.
   The current configuration is degrees. Do not inject non-zero steering until
   ros2can's expected unit has been verified.

3. Confirm the direct command route has one publisher per actuator.

   ```bash
   rostopic info /steering_requested_phy
   rostopic info /acceleration_requested_phy
   ```

4. Confirm CAN feedback is present.

   ```bash
   rostopic echo -n 1 /current_velocity_phy
   rostopic echo -n 1 /sending_angle
   rostopic echo -n 1 /vehicle_status
   ```

5. Set a physical test area, safety driver, independent emergency-stop route,
   and reviewed low-speed limit. Start with sensor faults or zero-valued
   actuator commands.

## Start

```bash
roslaunch adeye manager_real_world_sensor_faults.launch
rosrun adeye fault_test.py
```

Foxglove uses the existing `/vehicle_commands` topic with `std_msgs/String`.

## Software limits and timeout

The real-world launch sets conservative software caps:

| Fault value | Current cap |
| --- | ---: |
| Steering offset | 1.0 direct steering units |
| Steering saturation | 10.0 direct steering units |
| Steering oscillation | 0.5 direct steering units |
| Acceleration offset | 0.2 |
| Acceleration saturation | 0.5 |
| Acceleration oscillation | 0.1 |
| Any physical actuator fault duration | 5 seconds |

These are unvalidated software guardrails, not approved vehicle limits. Change
them only after a documented review of units, vehicle dynamics, and the test
safety case. The five-second timeout clears the fault state, but acceleration
timeout recovery is not yet independently verified at the output/CAN level.
The menu still sends its explicit reset command.

## Allowed physical menu scope

- Steering: options 3--6.
- Acceleration: options 8--11.
- GNSS/LiDAR sensor faults: options 16--21 after their routes are verified.

Options 7 (random steering), 12 (runaway acceleration), 15 (all tests), and
22 (combined localization input loss) are simulation-only. Options 1 and 2
are handled by the existing ros2can GUI command parser; verify their CAN
semantics separately before use.

Do not use direct `STEERING_command=<value>` or
`ACCELERATE_command=<value>` GUI commands during fault tests. They are handled
by ros2can outside the current fault-limit and timeout logic. Their protection
is an outstanding blocker.

## Verify a command reached the vehicle interface

Current topics can prove delivery through ROS but cannot by themselves prove
that CAN accepted a command. For each trial record:

1. GUI command: `rostopic echo /vehicle_commands`.
2. Faulted direct command:

   ```bash
   rostopic echo /steering_requested_phy
   rostopic echo /acceleration_requested_phy
   ```

3. CAN feedback: compare `/sending_angle`, `/current_velocity_phy`, and the
   relevant values in `/vehicle_status` with the expected response.

### Recommended ros2can acknowledgement implementation

Extend the existing `/vehicle_status` JSON publisher in ros2can rather than
adding a new ROS topic. Include, for each steering and acceleration command:

- received value and receipt timestamp;
- whether the controller was `ENGAGED` and accepted or rejected it;
- CAN transmit result/error code; and
- latest measured steering angle or longitudinal response timestamp.

Then a trial is acknowledged only when the direct command was observed, the
JSON reports accepted/transmitted, and measured CAN feedback changes within a
reviewed tolerance and time limit. Acceleration requires a vehicle-specific
feedback signal; velocity alone is not a complete acknowledgement.

## After every trial

1. Send menu option 14 (reset all).
2. Verify both direct actuator topics return to their baseline values.
3. Confirm the fault timeout did not log an unexpected expiry.
4. Record command, direct-topic value, CAN acknowledgement, feedback, and
   safety-system response.
