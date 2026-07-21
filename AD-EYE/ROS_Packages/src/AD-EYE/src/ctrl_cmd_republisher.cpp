#include <algorithm>
#include <cmath>
#include <exception>
#include <string>

#include <ros/ros.h>
#include <autoware_msgs/ControlCommandStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

ros::Publisher pub;

// ===== AD-EYE fault-injection change: physical steering faults =====
// Select the unit expected by ros2can on /steering_requested_phy before a
// physical test. The control command from Autoware is expressed in radians.
// const bool kSteeringRequestedPhyUsesRadians = true;
const bool kSteeringRequestedPhyUsesRadians = false;

const double kMaxSteeringAngleRadians = 0.85;
const double kRadiansToDegrees = 180.0 / M_PI;

double steering_offset = 0.0;
double steering_saturation = 0.0;
double steering_oscillation = 0.0;
bool steering_freeze_enabled = false;
bool has_frozen_steering = false;
double frozen_steering = 0.0;
double max_steering_offset = 1.0;
double max_steering_saturation = 10.0;
double max_steering_oscillation = 0.5;
double fault_timeout_s = 5.0;
ros::WallTime fault_deadline;
bool fault_deadline_active = false;
double last_unfaulted_steering = 0.0;
bool has_last_unfaulted_steering = false;

bool parseValue(const std::string& command, const std::string& prefix, double* value)
{
    if (command.compare(0, prefix.length(), prefix) != 0) {
        return false;
    }

    try {
        const double parsed = std::stod(command.substr(prefix.length()));
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

void vehicleCommandCallback(const std_msgs::String::ConstPtr& message)
{
    double value = 0.0;

    if (parseValue(message->data, "STEEROFFSET_command=", &value)) {
        if (std::abs(value) <= max_steering_offset) {
            steering_offset = value;
        } else {
            ROS_WARN("Rejected steering offset outside physical limit: %f", value);
        }
    } else if (message->data == "STEERFREEZE_command=1") {
        steering_freeze_enabled = true;
        has_frozen_steering = false;
    } else if (message->data == "STEERFREEZE_command=0") {
        steering_freeze_enabled = false;
        has_frozen_steering = false;
    } else if (parseValue(message->data, "STEERSAT_command=", &value)) {
        if (std::abs(value) <= max_steering_saturation) {
            steering_saturation = std::abs(value);
        } else {
            ROS_WARN("Rejected steering saturation outside physical limit: %f", value);
        }
    } else if (parseValue(message->data, "STEEROSC_command=", &value)) {
        if (std::abs(value) <= max_steering_oscillation) {
            steering_oscillation = value;
        } else {
            ROS_WARN("Rejected steering oscillation outside physical limit: %f", value);
        }
    } else if (message->data == "STEERRANDOM_command=1") {
        ROS_WARN("STEERRANDOM is not applied on the physical actuator path");
    }

    if (steering_offset != 0.0 || steering_saturation != 0.0 ||
        steering_oscillation != 0.0 || steering_freeze_enabled) {
        fault_deadline = ros::WallTime::now() + ros::WallDuration(fault_timeout_s);
        fault_deadline_active = true;
    } else {
        fault_deadline_active = false;
    }
}

bool clearExpiredSteeringFaults()
{
    if (!fault_deadline_active || ros::WallTime::now() < fault_deadline) {
        return false;
    }

    steering_offset = 0.0;
    steering_saturation = 0.0;
    steering_oscillation = 0.0;
    steering_freeze_enabled = false;
    has_frozen_steering = false;
    fault_deadline_active = false;
    ROS_ERROR("Physical steering fault timeout expired; reset to normal command.");
    return true;
}

double toSteeringRequestedPhy(double steering_radians)
{
    if (kSteeringRequestedPhyUsesRadians) {
        return steering_radians;
    }
    return steering_radians * kRadiansToDegrees;
}

void ctrlCmdCallback(const autoware_msgs::ControlCommandStamped::ConstPtr& control_cmd)
{
    clearExpiredSteeringFaults();
    double steering_radians = control_cmd->cmd.steering_angle;
    steering_radians = std::max(
        -kMaxSteeringAngleRadians,
        std::min(steering_radians, kMaxSteeringAngleRadians)
    );

    double steering_command = toSteeringRequestedPhy(steering_radians);
    last_unfaulted_steering = steering_command;
    has_last_unfaulted_steering = true;

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

    std_msgs::Float64 steering_cmd;
    steering_cmd.data = steering_command;
    pub.publish(steering_cmd);
}

void steeringFaultWatchdogCallback(const ros::WallTimerEvent&)
{
    if (clearExpiredSteeringFaults() && has_last_unfaulted_steering) {
        std_msgs::Float64 steering_cmd;
        steering_cmd.data = last_unfaulted_steering;
        pub.publish(steering_cmd);
    }
}
// ===== End AD-EYE fault-injection change =====

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ctrl_cmd_republisher");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    const double default_offset_limit =
        kSteeringRequestedPhyUsesRadians ? 0.02 : 1.0;
    const double default_saturation_limit =
        kSteeringRequestedPhyUsesRadians ? 0.20 : 10.0;
    const double default_oscillation_limit =
        kSteeringRequestedPhyUsesRadians ? 0.01 : 0.5;
    private_nh.param("max_steering_offset", max_steering_offset, default_offset_limit);
    private_nh.param("max_steering_saturation", max_steering_saturation, default_saturation_limit);
    private_nh.param("max_steering_oscillation", max_steering_oscillation, default_oscillation_limit);
    private_nh.param("fault_timeout_s", fault_timeout_s, 5.0);
    if (fault_timeout_s <= 0.0) {
        ROS_WARN("Invalid steering fault timeout; using 5 seconds.");
        fault_timeout_s = 5.0;
    }

    ros::Subscriber sub_ctrl_cmd = nh.subscribe<autoware_msgs::ControlCommandStamped>(
        "/ctrl_cmd", 1, ctrlCmdCallback
    );
    // AD-EYE fault-injection change: GUI fault-control subscription.
    ros::Subscriber sub_vehicle_commands = nh.subscribe<std_msgs::String>(
        "/vehicle_commands", 10, vehicleCommandCallback
    );

    pub = nh.advertise<std_msgs::Float64>("/steering_requested_phy", 1);
    ros::WallTimer fault_watchdog = nh.createWallTimer(
        ros::WallDuration(0.1), steeringFaultWatchdogCallback
    );

    ros::spin();

    return 0;
}
