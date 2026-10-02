#include <algorithm>
#include <cmath>
#include <exception>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <autoware_msgs/ControlCommandStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

ros::Publisher pub;
ros::Publisher fault_status_pub;
ros::Publisher steering_output_inhibit_pub;

// ===== AD-EYE fault-injection change: physical steering faults =====
// Autoware and the ros2can DBC both use radians. Degrees are a GUI-only unit.

const double kMaxSteeringAngleRadians = 0.85;

double steering_offset = 0.0;
double steering_saturation = 0.0;
double steering_oscillation = 0.0;
bool steering_freeze_enabled = false;
bool has_frozen_steering = false;
double frozen_steering = 0.0;
double max_steering_offset = M_PI / 180.0;
double max_steering_saturation = 10.0 * M_PI / 180.0;
double max_steering_oscillation = 0.5 * M_PI / 180.0;
double fault_timeout_s = 5.0;
double max_fault_timeout_s = 300.0;
double max_nominal_command_age_s = 0.5;
ros::WallTime fault_deadline;
bool fault_deadline_active = false;
std::string active_steering_fault;
double last_unfaulted_steering = 0.0;
bool has_last_unfaulted_steering = false;
ros::WallTime last_unfaulted_steering_time;

void publishSteeringFaultStatus(
    const std::string& event,
    const std::string& fault,
    bool active,
    double value,
    const std::string& detail
) {
    double remaining_s = 0.0;
    if (active && fault_deadline_active) {
        remaining_s = std::max(0.0, (fault_deadline - ros::WallTime::now()).toSec());
    }

    std_msgs::String status;
    std::ostringstream json;
    json << "{\"source\":\"steering\",\"event\":\"" << event
         << "\",\"fault\":\"" << fault << "\",\"active\":"
         << (active ? "true" : "false") << ",\"value\":" << value
         << ",\"timeout_s\":" << fault_timeout_s
         << ",\"remaining_s\":" << remaining_s
         << ",\"detail\":\"" << detail << "\"}";
    status.data = json.str();
    fault_status_pub.publish(status);
}

bool canActivateSteeringFault(const std::string& fault)
{
    return active_steering_fault.empty() || active_steering_fault == fault;
}

void activateSteeringFault(const std::string& fault, double value)
{
    active_steering_fault = fault;
    fault_deadline = ros::WallTime::now() + ros::WallDuration(fault_timeout_s);
    fault_deadline_active = true;
    publishSteeringFaultStatus("accepted", fault, true, value, "fault accepted");
}

void deactivateSteeringFault(const std::string& fault)
{
    if (active_steering_fault == fault) {
        active_steering_fault.clear();
        fault_deadline_active = false;
    }
    publishSteeringFaultStatus("reset", fault, false, 0.0, "fault reset");
}

bool parseValue(const std::string& command, const std::string& prefix, double* value)
{
    if (command.compare(0, prefix.length(), prefix) != 0) {
        return false;
    }

    try {
        const std::string value_text = command.substr(prefix.length());
        std::size_t consumed = 0;
        const double parsed = std::stod(value_text, &consumed);
        if (consumed != value_text.length()) {
            ROS_WARN("Ignoring steering fault command with trailing characters: %s", command.c_str());
            return false;
        }
        if (!std::isfinite(parsed)) {
            ROS_WARN("Ignoring non-finite steering fault command: %s", command.c_str());
            return false;
        }
        *value = parsed;
        return true;
    } catch (const std::exception&) {
        ROS_WARN("Ignoring invalid steering fault command: %s", command.c_str());
        return false;
    }
}

void physicalFaultConfigCallback(const std_msgs::String::ConstPtr& message)
{
    double value = 0.0;
    const std::string command = "PHYSICAL_FAULT_TIMEOUT_command";
    if (!parseValue(message->data, command + "=", &value)) {
        return;
    }

    if (!std::isfinite(value)) {
        publishSteeringFaultStatus(
            "rejected", command, false, 0.0, "timeout must be a finite number"
        );
    } else if (fault_deadline_active) {
        publishSteeringFaultStatus(
            "rejected", command, false, value, "cannot change timeout while a steering fault is active"
        );
    } else if (value <= 0.0 || value > max_fault_timeout_s) {
        publishSteeringFaultStatus(
            "rejected", command, false, value, "timeout outside configured limits"
        );
    } else {
        fault_timeout_s = value;
        publishSteeringFaultStatus(
            "configured", command, false, value, "physical fault timeout updated"
        );
    }
}

void vehicleCommandCallback(const std_msgs::String::ConstPtr& message)
{
    double value = 0.0;

    if (parseValue(message->data, "STEEROFFSET_command=", &value)) {
        const std::string fault = "STEEROFFSET_command";
        if (value == 0.0) {
            steering_offset = 0.0;
            deactivateSteeringFault(fault);
        } else if (!std::isfinite(value)) {
            publishSteeringFaultStatus("rejected", fault, false, 0.0, "value must be finite");
        } else if (std::abs(value) > max_steering_offset) {
            ROS_WARN("Rejected steering offset outside physical limit: %f", value);
            publishSteeringFaultStatus("rejected", fault, false, value, "outside physical limit");
        } else if (!canActivateSteeringFault(fault)) {
            publishSteeringFaultStatus("rejected", fault, false, value, "another steering fault is active");
        } else {
            steering_offset = value;
            activateSteeringFault(fault, value);
        }
    } else if (message->data == "STEERFREEZE_command=1") {
        const std::string fault = "STEERFREEZE_command";
        if (!canActivateSteeringFault(fault)) {
            publishSteeringFaultStatus("rejected", fault, false, 1.0, "another steering fault is active");
        } else {
            steering_freeze_enabled = true;
            has_frozen_steering = false;
            activateSteeringFault(fault, 1.0);
        }
    } else if (message->data == "STEERFREEZE_command=0") {
        steering_freeze_enabled = false;
        has_frozen_steering = false;
        deactivateSteeringFault("STEERFREEZE_command");
    } else if (parseValue(message->data, "STEERSAT_command=", &value)) {
        const std::string fault = "STEERSAT_command";
        if (value == 0.0) {
            steering_saturation = 0.0;
            deactivateSteeringFault(fault);
        } else if (!std::isfinite(value)) {
            publishSteeringFaultStatus("rejected", fault, false, 0.0, "value must be finite");
        } else if (value < 0.0 || value > max_steering_saturation) {
            ROS_WARN("Rejected steering saturation outside physical limit: %f", value);
            publishSteeringFaultStatus("rejected", fault, false, value, "outside physical limit");
        } else if (!canActivateSteeringFault(fault)) {
            publishSteeringFaultStatus("rejected", fault, false, value, "another steering fault is active");
        } else {
            steering_saturation = std::abs(value);
            activateSteeringFault(fault, steering_saturation);
        }
    } else if (parseValue(message->data, "STEEROSC_command=", &value)) {
        const std::string fault = "STEEROSC_command";
        if (value == 0.0) {
            steering_oscillation = 0.0;
            deactivateSteeringFault(fault);
        } else if (!std::isfinite(value)) {
            publishSteeringFaultStatus("rejected", fault, false, 0.0, "value must be finite");
        } else if (value < 0.0 || value > max_steering_oscillation) {
            ROS_WARN("Rejected steering oscillation outside physical limit: %f", value);
            publishSteeringFaultStatus("rejected", fault, false, value, "outside physical limit");
        } else if (!canActivateSteeringFault(fault)) {
            publishSteeringFaultStatus("rejected", fault, false, value, "another steering fault is active");
        } else {
            steering_oscillation = value;
            activateSteeringFault(fault, value);
        }
    } else if (message->data == "STEERRANDOM_command=1") {
        ROS_WARN("STEERRANDOM is not applied on the physical actuator path");
        publishSteeringFaultStatus(
            "rejected", "STEERRANDOM_command", false, 1.0, "simulation only"
        );
    }
}

bool clearExpiredSteeringFaults()
{
    if (!fault_deadline_active || ros::WallTime::now() < fault_deadline) {
        return false;
    }

    const std::string expired_fault = active_steering_fault;
    steering_offset = 0.0;
    steering_saturation = 0.0;
    steering_oscillation = 0.0;
    steering_freeze_enabled = false;
    has_frozen_steering = false;
    fault_deadline_active = false;
    active_steering_fault.clear();
    ROS_ERROR("Physical steering fault timeout expired; reset to normal command.");
    publishSteeringFaultStatus("timeout", expired_fault, false, 0.0, "hard timeout expired");
    return true;
}

void ctrlCmdCallback(const autoware_msgs::ControlCommandStamped::ConstPtr& control_cmd)
{
    clearExpiredSteeringFaults();
    double steering_radians = control_cmd->cmd.steering_angle;
    steering_radians = std::max(
        -kMaxSteeringAngleRadians,
        std::min(steering_radians, kMaxSteeringAngleRadians)
    );

    double steering_command = steering_radians;
    last_unfaulted_steering = steering_command;
    has_last_unfaulted_steering = true;
    last_unfaulted_steering_time = ros::WallTime::now();

    if (steering_freeze_enabled) {
        if (!has_frozen_steering) {
            frozen_steering = steering_command;
            has_frozen_steering = true;
        }
        steering_command = frozen_steering;
    }

    steering_command += steering_offset;

    if (steering_saturation > 0.0) {
        steering_command = std::max(
            -steering_saturation,
            std::min(steering_command, steering_saturation)
        );
    }

    if (steering_oscillation != 0.0) {
        steering_command += steering_oscillation * std::sin(ros::Time::now().toSec() * 2.0);
    }

    // Final physical boundary: never publish outside the ros2can DBC range.
    steering_command = std::max(
        -kMaxSteeringAngleRadians,
        std::min(steering_command, kMaxSteeringAngleRadians)
    );

    std_msgs::Float64 steering_cmd;
    steering_cmd.data = steering_command;
    pub.publish(steering_cmd);
}

void steeringFaultWatchdogCallback(const ros::WallTimerEvent&)
{
    if (clearExpiredSteeringFaults()) {
        const bool nominal_is_fresh = has_last_unfaulted_steering &&
            (ros::WallTime::now() - last_unfaulted_steering_time).toSec() <=
                max_nominal_command_age_s;
        if (nominal_is_fresh) {
            std_msgs::Float64 steering_cmd;
            steering_cmd.data = last_unfaulted_steering;
            pub.publish(steering_cmd);
            publishSteeringFaultStatus(
                "recovery", "steering_output", false, last_unfaulted_steering,
                "fresh nominal steering republished after timeout"
            );
        } else {
            std_msgs::Bool inhibit;
            inhibit.data = true;
            steering_output_inhibit_pub.publish(inhibit);
            publishSteeringFaultStatus(
                "inhibited", "steering_output", false, 0.0,
                "nominal steering stale or absent; ros2can steering output inhibited"
            );
        }
    } else if (fault_deadline_active) {
        double active_value = 1.0;
        if (active_steering_fault == "STEEROFFSET_command") {
            active_value = steering_offset;
        } else if (active_steering_fault == "STEERSAT_command") {
            active_value = steering_saturation;
        } else if (active_steering_fault == "STEEROSC_command") {
            active_value = steering_oscillation;
        }
        publishSteeringFaultStatus(
            "active", active_steering_fault, true, active_value, "fault active"
        );
    }
}
// ===== End AD-EYE fault-injection change =====

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ctrl_cmd_republisher");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    const double default_offset_limit = M_PI / 180.0;
    const double default_saturation_limit = 10.0 * M_PI / 180.0;
    const double default_oscillation_limit = 0.5 * M_PI / 180.0;
    private_nh.param("max_steering_offset", max_steering_offset, default_offset_limit);
    private_nh.param("max_steering_saturation", max_steering_saturation, default_saturation_limit);
    private_nh.param("max_steering_oscillation", max_steering_oscillation, default_oscillation_limit);
    private_nh.param("fault_timeout_s", fault_timeout_s, 5.0);
    private_nh.param("max_fault_timeout_s", max_fault_timeout_s, 300.0);
    private_nh.param("max_nominal_command_age_s", max_nominal_command_age_s, 0.5);
    if (!std::isfinite(max_steering_offset) || max_steering_offset <= 0.0) {
        ROS_WARN("Invalid maximum steering offset; using 1 degree in radians.");
        max_steering_offset = default_offset_limit;
    }
    if (!std::isfinite(max_steering_saturation) || max_steering_saturation <= 0.0 ||
        max_steering_saturation > kMaxSteeringAngleRadians) {
        ROS_WARN("Invalid maximum steering saturation; using 10 degrees in radians.");
        max_steering_saturation = default_saturation_limit;
    }
    if (!std::isfinite(max_steering_oscillation) || max_steering_oscillation <= 0.0) {
        ROS_WARN("Invalid maximum steering oscillation; using 0.5 degree in radians.");
        max_steering_oscillation = default_oscillation_limit;
    }
    if (max_fault_timeout_s <= 0.0) {
        ROS_WARN("Invalid maximum steering fault timeout; using 300 seconds.");
        max_fault_timeout_s = 300.0;
    }
    if (fault_timeout_s <= 0.0) {
        ROS_WARN("Invalid steering fault timeout; using 5 seconds.");
        fault_timeout_s = 5.0;
    } else if (fault_timeout_s > max_fault_timeout_s) {
        ROS_WARN("Steering fault timeout exceeds maximum; using maximum.");
        fault_timeout_s = max_fault_timeout_s;
    }
    if (!std::isfinite(max_nominal_command_age_s) || max_nominal_command_age_s <= 0.0) {
        ROS_WARN("Invalid nominal steering command maximum age; using 0.5 seconds.");
        max_nominal_command_age_s = 0.5;
    }

    ros::Subscriber sub_ctrl_cmd = nh.subscribe<autoware_msgs::ControlCommandStamped>(
        "/ctrl_cmd", 1, ctrlCmdCallback
    );
    // AD-EYE fault-injection change: GUI fault-control subscription.
    ros::Subscriber sub_vehicle_commands = nh.subscribe<std_msgs::String>(
        "/vehicle_commands", 10, vehicleCommandCallback
    );
    ros::Subscriber sub_physical_fault_config = nh.subscribe<std_msgs::String>(
        "/adeye/physical_fault_config", 10, physicalFaultConfigCallback
    );

    pub = nh.advertise<std_msgs::Float64>("/steering_requested_phy", 1);
    fault_status_pub = nh.advertise<std_msgs::String>(
        "/adeye/physical_fault_status", 10
    );
    steering_output_inhibit_pub = nh.advertise<std_msgs::Bool>(
        "/adeye/steering_output_inhibit", 1
    );
    ros::WallTimer fault_watchdog = nh.createWallTimer(
        ros::WallDuration(0.1), steeringFaultWatchdogCallback
    );

    ros::spin();

    return 0;
}
