"""List features for the configuration panel without reading or changing ROS or files.

Listed defaults are examples, not readings from the running system.
"""
import copy


RESET_POLICY = {
    "name": "Reset configuration - remove this panel's overrides",
    "baseline": "effective configuration without Test Configuration overrides",
    "baseline_evidence": "resolved launch arguments plus component-confirmed boot settings and separately owned runtime changes",
    "persistence": "session_only_unless_explicitly_saved",
    "restart": "manual",
    "scope": "Test Configuration owned overrides, drafts and pending next-launch changes only",
    "saved_override": "offer explicit removal of this panel's saved startup override; never delete unrelated profiles",
    "no_changes": "no requests, parameter writes or vehicle actions",
    "active_test": "reject affected changes until the operator stops the test or cancels its pending trigger",
    "conflict": "do not overwrite changes owned by another panel or publisher",
    "missing_evidence": "report unknown; never substitute GUI defaults or stale ROS parameters",
    "partial_reset": "report per-component pending/failed state; never claim complete",
    "restart_required": "show reset prepared until a manually restarted component confirms the baseline",
    "boot_identity": "discard old applied-state claims when manager/component boot identity changes",
    "never_restore": ["active_fault", "old_actuator_command", "pending_fault_timer",
                      "goal", "replay_playback", "ad_mode", "recovery_hold_release"],
    "never_touch": ["oem_emergency_handling", "original_can_emergency_handling",
                    "unrelated_ros_parameters", "recorded_bags", "external_saved_configuration"],
}


def _feature(identifier, label, group, owner, current_mode, planned_mode,
             disabled, hint=None, dependencies=(), bindings=(), notes="", references=()):
    return {
        "id": identifier, "label": label, "group": group, "owner": owner,
        "kind": "feature", "current_change_mode": current_mode,
        "planned_change_mode": planned_mode, "default_hint": hint,
        "default_hint_is_applied_state": False,
        "baseline_source": "component_confirmed_effective_configuration",
        "dependencies": list(dependencies), "startup_bindings": list(bindings),
        "disabled_behavior": disabled, "notes": notes,
        "reset_policy": "remove_owned_override", "references": list(references),
    }


def _launch(argument):
    return "launch/manager_real_world.launch:arg:" + argument


def _features():
    restart = "manual_restart"
    runtime = "runtime_with_new_backend_handler"
    return [
        _feature("steering_proxy", "Steering proxy", "Steering", "steering_fault_proxy", restart, restart,
                 "Node absent. Launch must route the selected producer/replay to the correct output instead.",
                 True, bindings=[_launch("enable_steering_proxy")],
                 notes="Removing the proxy is not the same as bypassing faults. Do not kill the node to implement pass-through.",
                 references=["src/steering_fault_proxy.cpp", "launch/manager_real_world_base.launch"]),
        _feature("acceleration_proxy", "Acceleration proxy", "Acceleration", "recorded_acceleration_fault_proxy", restart, restart,
                 "Node absent. Launch must restore a complete command route.", True,
                 bindings=[_launch("enable_acceleration_proxy")], references=["src/recorded_acceleration_fault_proxy.cpp"]),
        _feature("steering_fault_processing", "Steering fault processing", "Steering", "steering_fault_proxy", restart, runtime,
                 "Existing startup-off proxy forwards nominal input without subscribing to fault commands/configuration.", True,
                 dependencies=["steering_proxy"], bindings=[_launch("enable_steering_faults"), "private:enable_fault_injection"],
                 notes="Runtime bypass needs a new handler; runtime re-enable from startup-off needs subscriptions added without activating faults.",
                 references=["src/steering_fault_proxy.cpp"]),
        _feature("acceleration_fault_processing", "Acceleration fault processing", "Acceleration", "recorded_acceleration_fault_proxy", restart, runtime,
                 "Existing startup-off proxy forwards nominal input without fault-command/config subscriptions.", True,
                 dependencies=["acceleration_proxy"], bindings=[_launch("enable_acceleration_faults"), "private:enable_fault_injection"],
                 references=["src/recorded_acceleration_fault_proxy.cpp"]),
        _feature("lidar_processing", "LiDAR fault path", "LiDAR", "SensorFaultInjectionManager", restart, restart,
                 "Current enable_lidar=false creates no LiDAR relay; another startup route must own the output.",
                 bindings=["sensor_private:enable_lidar", "sensor_private:lidar_input_topic", "sensor_private:lidar_output_topic"],
                 notes="Future runtime fault bypass must be a SEPARATE switch that keeps forwarding. This startup switch is not a bypass.",
                 references=["src/fault_injections/SensorFaultInjectionManager.py", "launch/manager_real_world_lidar_faults.launch"]),
        _feature("gnss_processing", "GNSS fault path", "GNSS", "SensorFaultInjectionManager", restart, restart,
                 "Current enable_gnss=false creates no GNSS relay. Startup must restore the correct broadcaster input.",
                 bindings=["sensor_private:enable_gnss", "sensor_private:gnss_input_topic", "sensor_private:gnss_output_topic"],
                 notes="GNSS is inventory only for now, not a recommendation to add it to live tests.",
                 references=["src/fault_injections/SensorFaultInjectionManager.py", "launch/manager_real_world_gnss_faults.launch"]),
        _feature("lidar_fault_processing", "Bypass LiDAR faults, keep forwarding", "LiDAR", "SensorFaultInjectionManager", "not_supported", runtime,
                 "Planned: reject LiDAR fault activation but forward incoming scans unchanged.",
                 dependencies=["lidar_processing"], notes="No such runtime switch exists yet. Not equivalent to enable_lidar=false.",
                 references=["src/fault_injections/SensorFaultInjectionManager.py"]),
        _feature("gnss_fault_processing", "Bypass GNSS faults, keep forwarding", "GNSS", "SensorFaultInjectionManager", "not_supported", runtime,
                 "Planned: reject GNSS fault activation but forward fixes unchanged.", dependencies=["gnss_processing"],
                 references=["src/fault_injections/SensorFaultInjectionManager.py"]),
        _feature("points_relay", "Normal points_raw relay", "LiDAR", "roslaunch/topic_tools", restart, restart,
                 "No normal relay; ensure the sensor proxy or another authorised route owns the output.", True,
                 bindings=[_launch("enable_default_points_raw_relay")],
                 notes="Default output routes must not run both the relay and sensor proxy as competing publishers.",
                 references=["launch/manager_real_world.launch"]),
        _feature("recorded_steering", "Recorded steering input", "Routing", "roslaunch", restart, restart,
                 "Select live steering producer and its route instead of replay input.", False,
                 bindings=[_launch("use_recorded_inputs"), _launch("use_recorded_steering")],
                 notes="use_recorded_inputs supplies defaults for both axes; axis-specific launch arguments may override it.",
                 references=["launch/manager_real_world.launch"]),
        _feature("recorded_acceleration", "Recorded acceleration input", "Routing", "roslaunch", restart, restart,
                 "Select live acceleration producer and its route instead of replay input.", False,
                 bindings=[_launch("use_recorded_inputs"), _launch("use_recorded_acceleration")],
                 references=["launch/manager_real_world.launch"]),
        _feature("goal_monitor", "Goal monitoring", "Goal handling", "physical_goal_stop_monitor", restart, restart,
                 "Standalone monitoring disabled; enabling goal-stop action still starts this monitor.", False,
                 bindings=[_launch("enable_goal_stop_monitor")],
                 notes="Effective monitor presence is enable_goal_stop_monitor OR enable_goal_stop, not just this flag.",
                 references=["launch/manager_real_world.launch", "src/physical_goal_stop_monitor.py"]),
        _feature("goal_stop_action", "Goal-stop action", "Goal handling", "goal monitor + producers + proxies + ros2can", restart, restart,
                 "No goal-based speed limiting from startup. Monitoring can run separately.", False,
                 bindings=[_launch("enable_goal_stop"), "private:enable_speed_control", "private:enable_goal_stop_priority"],
                 notes="Live-only action. A runtime implementation would need coordinated consumers and explicit clearing of latched speed limits; not a parameter write.",
                 references=["src/physical_goal_stop_monitor.py", "launch/manager_real_world.launch"]),
        _feature("manual_recovery", "Manual software recovery availability", "Recovery", "manual_vehicle_recovery + RecoveryGate + ros2can", restart, restart,
                 "Recovery request service/handlers are not enabled. Does not clear an existing recovery hold.", False,
                 bindings=[_launch("enable_manual_recovery"), "global:/adeye/enable_manual_recovery"],
                 notes="The recovery operation itself is a runtime service, not a configuration switch or reset-to-defaults action.",
                 references=["src/manual_vehicle_recovery.py", "include/recovery_gate.h"]),
        _feature("recovery_monitor", "Recovery information monitor", "Diagnostics", "vehicle_recovery_monitor", restart, restart,
                 "No assessment node; CAN/OEM handling remains unchanged.", True,
                 bindings=[_launch("enable_vehicle_recovery_monitor")], references=["launch/vehicle_recovery_monitor.launch"]),
        _feature("recording", "Physical command bag recording", "Recording", "record_physical_commands", restart, restart,
                 "No manager-started recorder. Existing bag files are never removed.", False,
                 bindings=[_launch("record_physical_commands")],
                 notes="Recorder starts a subprocess at construction and only has shutdown cleanup, not a runtime recording service.",
                 references=["src/record_physical_commands.py"]),
        _feature("fault_timeout_enabled", "Actuator fault timeout enabled", "Fault settings", "actuator proxies", restart, restart,
                 "Actuator faults do not expire through their ordinary fault timer. Continuous-send duration is separate.", True,
                 bindings=[_launch("enable_physical_fault_timeout"), "private:enable_fault_timeout"],
                 notes="Does not disable sensor timeout: the sensor manager has its own always-used fault deadline.",
                 references=["src/steering_fault_proxy.cpp", "src/recorded_acceleration_fault_proxy.cpp", "src/fault_injections/SensorFaultInjectionManager.py"]),
        _feature("fault_timeout_duration", "Ordinary fault timeout duration", "Fault settings", "actuator proxies + sensor manager", "runtime_idle_only", "runtime_idle_only",
                 "Not an on/off setting. Restore only this panel's owned duration override.", 5.0,
                 bindings=["config:PHYSICAL_FAULT_TIMEOUT_command"],
                 notes="Shared /adeye/physical_fault_config command can update several nodes, with partial rejection if one is active. Requires per-component applied evidence; no atomic transaction exists.",
                 references=["src/steering_fault_proxy.cpp", "src/recorded_acceleration_fault_proxy.cpp", "src/fault_injections/SensorFaultInjectionManager.py"]),
        _feature("fault_ranges", "Fault input min/max settings", "Fault settings", "actuator proxies + sensor manager", "runtime_idle_only", "runtime_idle_only",
                 "Not an enable switch. Restore only owned range overrides within component bounds.",
                 bindings=["config:FAULT_LIMITS_command", "config:FAULT_LIMITS_QUERY_command"],
                 notes="Range configuration is not range enforcement. Many checks are disabled in code. Actuator steering backend ranges use radians; GUI uses degrees.",
                 references=["src/steering_fault_proxy.cpp", "src/recorded_acceleration_fault_proxy.cpp", "src/fault_injections/SensorFaultInjectionManager.py"]),
        _feature("proxy_tuning", "Proxy rates, limits, frequency and recovery tuning", "Fault settings", "actuator proxies", restart, restart,
                 "No off state; settings are cached at construction.", references=["src/steering_fault_proxy.cpp", "src/recorded_acceleration_fault_proxy.cpp"]),
        _feature("allow_direct_steering", "Allow direct steering commands in ros2can", "Direct controls", "ros2can", restart, restart,
                 "Direct STEERING_command is not accepted; other routes are not disabled by this permission.", True,
                 bindings=["ros2can_private:allow_direct_steering_commands"], references=["ros2can:ros_canlib/src/vehicle/src/main.cpp"]),
        _feature("allow_direct_acceleration", "Allow direct acceleration commands in ros2can", "Direct controls", "ros2can", restart, restart,
                 "Direct ACCELERATE_command is not accepted; other routes remain separate.", True,
                 bindings=["ros2can_private:allow_direct_acceleration_commands"], references=["ros2can:ros_canlib/src/vehicle/src/main.cpp"]),
        _feature("gui_direct_control_interlocks", "Disable direct GUI commands during faults", "GUI", "Steering.tsx + Acceleration.tsx", "code_change", "runtime_with_new_gui_handler",
                 "Existing code-level option lets direct controls remain available during faults.", True,
                 notes="Hardcoded GUI constants; changing ROS parameters does not update them. A change currently requires rebuilding the extension.",
                 references=["gui:src/Panels/Steering/Steering.tsx", "gui:src/Panels/Acceleration/Acceleration.tsx"]),
        _feature("range_enforcement", "Optional fault-input range enforcement", "Fault settings", "GUI + proxies + sensor manager", "code_change", "not_planned",
                 "Configured input ranges are not generally enforced; finite/mode/geometry/output rules can still apply.", False,
                 notes="C++ constexpr/Python and GUI constants, not live flags. No automatic re-enabling is in scope.",
                 references=["src/steering_fault_proxy.cpp", "src/recorded_acceleration_fault_proxy.cpp", "src/fault_injections/SensorFaultInjectionManager.py"]),
        _feature("command_watchdog_actions", "Disabled command-age/output-inhibit actions", "Diagnostics", "proxies + ros2can", "code_change", "not_planned",
                 "Keep currently disabled automatic intervention paths disabled; diagnostics remain information only.", False,
                 notes="Some compatibility parameters remain readable but do not enable the commented-out pipeline. Excluded from editable switches.",
                 references=["ros2can:ros_canlib/src/vehicle/src/main.cpp", "src/steering_fault_proxy.cpp", "src/recorded_acceleration_fault_proxy.cpp"]),
        _feature("random_triggers", "Pending random fault schedules", "GUI", "RandomDelayFaults + RandomGoalProgressFaults", "runtime_gui_only", "runtime_gui_only",
                 "Cancel a pending GUI schedule explicitly; do not restore it during configuration reset.",
                 notes="Arming/cancelling exists now, but feature-wide availability needs new GUI/backend handling. Active faults require their own stop.",
                 references=["gui:src/Panels/RandomDelayFaults/controller.ts", "gui:src/Panels/RandomDelayFaults/goalProgress.ts"]),
        _feature("fault_logging", "Fault log writing", "Diagnostics", "manager/FaultInjectionManager", "not_supported", "runtime_with_new_backend_handler",
                 "Planned: stop optional log writing without changing fault handling or required status reporting.",
                 notes="Currently constructed unconditionally by manager; no live logging enable switch exists. This phase does not open, prune or delete log files.",
                 references=["src/manager.py", "src/fault_injections/FaultInjectionManager.py"]),
        _feature("fault_log_settings", "Fault log directory, rotation and verbosity", "Diagnostics", "FaultInjectionManager", restart, restart,
                 "No off state. Current logger settings are read once at construction.",
                 bindings=["manager_private:fault_log_dir", "manager_private:fault_log_max_bytes", "manager_private:fault_log_keep_files",
                           "manager_private:fault_log_heartbeat_s", "manager_private:fault_log_value_change_min_interval_s",
                           "manager_private:fault_log_compress_rotated"],
                 references=["src/fault_injections/FaultInjectionManager.py"]),
        _feature("required_status_reporting", "Required fault/configuration status reporting", "Diagnostics", "proxies + sensor manager + configuration backend", "not_supported", "not_planned",
                 "No disable switch planned: keep acknowledgements and applied-state reporting available.",
                 notes="GUI interlocks, recovery and configuration visibility rely on status. Optional logging verbosity is a separate setting.",
                 references=["src/steering_fault_proxy.cpp", "src/fault_injections/SensorFaultInjectionManager.py"]),
    ]


def _faults():
    labels = {
        "STEERFREEZE": "Steering freeze", "STEEROFFSET": "Steering offset",
        "STEERSAT": "Steering saturation", "STEEROSC": "Steering oscillation",
        "STEERCONSTANT": "Continuous steering", "ACCELFREEZE": "Acceleration freeze",
        "ACCELOFFSET": "Acceleration offset", "ACCELSAT": "Acceleration saturation",
        "ACCELOSC": "Acceleration oscillation", "ACCELCONSTANT": "Continuous acceleration",
        "GNSS_DROPOUT": "GNSS dropout", "GNSS_NO_FIX": "GNSS no-fix status",
        "GNSS_EAST_BIAS_M": "GNSS east bias", "GNSS_NORTH_BIAS_M": "GNSS north bias",
        "LIDAR_DROP_EVERY_N": "LiDAR periodic scan dropout",
        "LIDAR_TIMESTAMP_OFFSET_S": "LiDAR timestamp offset", "LIDAR_FREEZE": "LiDAR frozen scan",
        "LIDAR_DELAY_S": "LiDAR real message delay", "LIDAR_SECTOR_DEG": "LiDAR partial scan loss",
        "LIDAR_RANGE_M": "LiDAR reduced detection range", "LIDAR_POINT_LOSS_PCT": "LiDAR random point loss",
        "LIDAR_NOISE_STDDEV_M": "LiDAR measurement noise", "LIDAR_KEEP_EVERY_N": "LiDAR reduced scan rate",
    }
    for axis in ("X", "Y", "Z"):
        labels["LIDAR_TRANSLATE_" + axis + "_M"] = "LiDAR position offset " + axis
    for axis in ("ROLL", "PITCH", "YAW"):
        labels["LIDAR_" + axis + "_DEG"] = "LiDAR " + axis.lower() + " offset"
    groups = [
        ("Steering", "steering_fault_processing", ["STEERFREEZE", "STEEROFFSET", "STEERSAT", "STEEROSC", "STEERCONSTANT"]),
        ("Acceleration", "acceleration_fault_processing", ["ACCELFREEZE", "ACCELOFFSET", "ACCELSAT", "ACCELOSC", "ACCELCONSTANT"]),
        ("GNSS", "gnss_fault_processing", ["GNSS_DROPOUT", "GNSS_NO_FIX", "GNSS_EAST_BIAS_M", "GNSS_NORTH_BIAS_M"]),
        ("LiDAR", "lidar_fault_processing", ["LIDAR_DROP_EVERY_N", "LIDAR_TIMESTAMP_OFFSET_S", "LIDAR_FREEZE", "LIDAR_DELAY_S",
            "LIDAR_SECTOR_DEG", "LIDAR_RANGE_M", "LIDAR_POINT_LOSS_PCT", "LIDAR_NOISE_STDDEV_M", "LIDAR_KEEP_EVERY_N",
            "LIDAR_TRANSLATE_X_M", "LIDAR_TRANSLATE_Y_M", "LIDAR_TRANSLATE_Z_M", "LIDAR_ROLL_DEG", "LIDAR_PITCH_DEG", "LIDAR_YAW_DEG"]),
    ]
    result = []
    for group, dependency, commands in groups:
        for command in commands:
            item = _feature("fault." + command, labels[command], group,
                            "actuator proxy" if group in ("Steering", "Acceleration") else "SensorFaultInjectionManager",
                            "not_supported", "runtime_with_new_backend_handler",
                            "Planned: reject new activations from every publisher; keep nominal forwarding. Require an affected active/pending test to be stopped first.",
                            dependencies=[dependency], references=["gui:src/Panels/FaultInjection/definitions.ts"])
            item.update({"kind": "fault_availability", "command": command + "_command",
                         "activation_runtime_supported": True,
                         "notes": "Activation/stop exists, but there is no individual availability switch. Availability inherits the running component, never default all-on. Continuous sends have separate duration controls."})
            result.append(item)
    return result


_REGISTRY = {
    "schema_version": 1,
    "phase": "inventory_only",
    "applies_configuration": False,
    "reset_policy": RESET_POLICY,
    "features": _features() + _faults(),
    "profiles": [{"id": name, "launch_file": "manager_real_world_" + name + ".launch",
                  "baseline": "resolve selected module, nested includes and explicit arguments; do not infer applied state from this name"}
                 for name in ["base", "proxy_only", "steering_faults", "acceleration_faults", "steering_acceleration_faults",
                              "sensor_faults", "lidar_faults", "gnss_faults", "all_faults"]],
    "excluded_controls": ["HL_command", "WLOCK_command", "STEERRANDOM_command", "ACCELRUNAWAY_command"],
    "exclusion_reason": "Direct vehicle controls and simulation-only faults are not physical proxy-availability switches.",
    "constraints": [
        "Do not introduce a mandatory test order or pass/fail prerequisites.",
        "Preserve normal forwarding when fault processing is bypassed.",
        "Do not route both the normal relay and sensor proxy to the same output.",
        "A reset removes only owned configuration, never restores a command or activates AD.",
        "Backend acknowledgements/boot identity and revisioned ownership must be added before editable controls.",
        "Shared timeout messages can partially apply; report each component, not just the first acknowledgement.",
    ],
}


def get_registry():
    """Return a separate copy of the feature list. It does not describe current node settings."""
    return copy.deepcopy(_REGISTRY)


def validate_registry(registry):
    """Check the feature list before it is sent to the panel."""
    features = registry["features"]
    by_id = {item["id"]: item for item in features}
    if len(by_id) != len(features):
        raise ValueError("duplicate feature id")
    valid_modes = {"manual_restart", "runtime_with_new_backend_handler", "not_supported",
                   "runtime_idle_only", "code_change", "runtime_with_new_gui_handler", "runtime_gui_only", "not_planned"}
    visiting, visited = set(), set()
    def visit(identifier):
        if identifier in visiting:
            raise ValueError("cyclic dependency: " + identifier)
        if identifier in visited:
            return
        if identifier not in by_id:
            raise ValueError("unknown dependency: " + identifier)
        visiting.add(identifier)
        item = by_id[identifier]
        for name in ("label", "group", "owner", "disabled_behavior", "baseline_source", "reset_policy"):
            if not item.get(name):
                raise ValueError("missing feature metadata: " + identifier + ":" + name)
        for name in ("current_change_mode", "planned_change_mode"):
            if item[name] not in valid_modes:
                raise ValueError("unknown change mode: " + item[name])
        if item["default_hint_is_applied_state"] is not False:
            raise ValueError("defaults cannot stand in for applied state")
        for dependency in item["dependencies"]:
            visit(dependency)
        visiting.remove(identifier)
        visited.add(identifier)
    for identifier in by_id:
        visit(identifier)
    commands = [item["command"] for item in features if item["kind"] == "fault_availability"]
    if len(commands) != len(set(commands)):
        raise ValueError("duplicate fault command")
    if set(commands).intersection(registry["excluded_controls"]):
        raise ValueError("excluded control exposed as a fault switch")
    if registry["applies_configuration"] is not False:
        raise ValueError("Phase 1 must not apply configuration")
    policy = registry["reset_policy"]
    if policy["persistence"] != "session_only_unless_explicitly_saved" or policy["restart"] != "manual":
        raise ValueError("persistence and restart policy changed")
    if not set(RESET_POLICY["never_restore"]).issubset(policy["never_restore"]):
        raise ValueError("reset must not restore test/vehicle actions")
    return True
