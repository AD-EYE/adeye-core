"""Report configuration problems only. Listed ROS connections do not prove the car test works."""


def expected_routes(settings):
    routes = []
    for axis, producer, live, replay in (
            ("steering", "/ctrl_cmd_republisher", "/steering_fault_proxy_live", "/steering_fault_proxy_recorded"),
            ("acceleration", "/vehicle_controller", "/acceleration_fault_proxy_live", "/recorded_acceleration_fault_proxy")):
        output = "/%s_requested_phy" % axis
        proxy, recorded = settings[axis + "_proxy"], settings["recorded_" + axis]
        if proxy:
            node = replay if recorded else live
            source = ("/replay/" if recorded else "/nominal/") + axis + "_requested_phy"
            routes.append(dict(id=axis + ".input", topic=source, publisher=None if recorded else producer, subscriber=node))
            routes.append(dict(id=axis + ".output", topic=output, publisher=node, subscriber="/vi_controller"))
        else:
            routes.append(dict(id=axis + ".output", topic=output, publisher=producer, subscriber="/vi_controller"))
    routes.append(dict(id="lidar.output", topic="/points_raw",
                       publisher="/sensor_fault_injection_manager" if settings["lidar_path"] else "/points_raw_relay", subscriber=None))
    if settings["lidar_path"]:
        routes.append(dict(id="lidar.input", topic="/os_cloud_node/points", publisher=None, subscriber="/sensor_fault_injection_manager"))
    if settings["gnss_path"]:
        routes.append(dict(id="gnss.input", topic="/fix", publisher=None, subscriber="/sensor_fault_injection_manager"))
        routes.append(dict(id="gnss.output", topic="/adeye/fault_injection/fix", publisher="/sensor_fault_injection_manager", subscriber=None))
    return routes


def diagnose(system_state, startup=None, availability=None, errors=None):
    checks = []
    def add(identifier, state, detail): checks.append(dict(id=identifier, state=state, detail=detail))
    for name, error in sorted((errors or {}).items()):
        add("service." + name, "warning", name + " unavailable: " + str(error))
    settings = startup.get("running_launch") if startup else None
    if system_state is None:
        add("graph", "unknown", "ROS graph unavailable; node and route checks cannot be made")
    elif settings is None:
        add("launch", "unknown", "No complete configured-launch snapshot. Expected routes are unknown; defaults are not substituted")
    else:
        publishers = {topic: set(nodes) for topic, nodes in system_state[0]}
        subscribers = {topic: set(nodes) for topic, nodes in system_state[1]}
        for route in expected_routes(settings):
            topic = route["topic"]
            pubs, subs = publishers.get(topic, set()), subscribers.get(topic, set())
            missing = []
            if route["publisher"] and route["publisher"] not in pubs: missing.append("expected publisher " + route["publisher"] + " is not registered")
            if not pubs: missing.append("no publisher is registered (a replay bag may not be running)")
            if route["subscriber"] and route["subscriber"] not in subs: missing.append("expected subscriber " + route["subscriber"] + " is not registered")
            if len(pubs) > 1: missing.append("multiple publishers: " + ", ".join(sorted(pubs)))
            add(route["id"], "warning" if missing else "observed", topic + ": " + ("; ".join(missing) if missing else "expected registrations found; message flow and command acceptance are not verified"))
    if availability:
        for command, reason in sorted(availability.get("unconfirmed", {}).items()):
            add("availability." + command, "unknown", command + ": " + reason)
        if system_state is not None:
            registered = {node for section in system_state for _, nodes in section for node in nodes}
            components = {entry["component"] for entry in availability.get("commands", {}).values()}
            for component in sorted(components - registered):
                add("component." + component, "warning", "Stored acknowledgement from " + component + " exists, but the component is not listed in the graph. Do not treat it as proof the component is running")
    if startup:
        if startup["disk_error"]: add("startup.file", "warning", startup["disk_error"])
        if startup["validation_errors"]: add("startup.draft", "warning", "; ".join(startup["validation_errors"]))
        if startup["restart_required"]: add("startup.restart", "unknown", "Next-launch settings differ from the captured launch, or the launch is unknown. A manual restart and new observations are needed")
    return dict(information_only=True, checks=checks,
                meaning="No automatic stop, reset, routing change or fault cancellation. Observed means registrations only, never physical-car readiness.")
