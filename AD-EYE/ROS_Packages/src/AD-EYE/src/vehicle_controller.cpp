#include <algorithm>
#include <cmath>
#include <exception>
#include <sstream>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include "PID.h"
// #include <PID.h>
#include <functional>

std::string topic_acceleration_request;
std::string topic_steering_request;
ros::Publisher acceleration_fault_status_pub;

// ===== AD-EYE fault-injection change: physical acceleration faults =====
double acceleration_offset = 0.0;
double acceleration_saturation = 0.0;
double acceleration_oscillation = 0.0;
bool acceleration_freeze_enabled = false;
bool has_frozen_acceleration = false;
double frozen_acceleration = 0.0;
double max_acceleration_offset = 0.2;
double max_acceleration_saturation = 0.5;
double max_acceleration_oscillation = 0.1;
double fault_timeout_s = 5.0;
double max_fault_timeout_s = 300.0;
ros::WallTime fault_deadline;
bool fault_deadline_active = false;
std::string active_acceleration_fault;
double last_unfaulted_acceleration = 0.0;
bool has_last_unfaulted_acceleration = false;

void publishAccelerationFaultStatus(
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
    json << "{\"source\":\"acceleration\",\"event\":\"" << event
         << "\",\"fault\":\"" << fault << "\",\"active\":"
         << (active ? "true" : "false") << ",\"value\":" << value
         << ",\"timeout_s\":" << fault_timeout_s
         << ",\"remaining_s\":" << remaining_s
         << ",\"detail\":\"" << detail << "\"}";
    status.data = json.str();
    acceleration_fault_status_pub.publish(status);
}

bool canActivateAccelerationFault(const std::string& fault)
{
    return active_acceleration_fault.empty() || active_acceleration_fault == fault;
}

void activateAccelerationFault(const std::string& fault, double value)
{
    active_acceleration_fault = fault;
    fault_deadline = ros::WallTime::now() + ros::WallDuration(fault_timeout_s);
    fault_deadline_active = true;
    publishAccelerationFaultStatus("accepted", fault, true, value, "fault accepted");
}

void deactivateAccelerationFault(const std::string& fault)
{
    if (active_acceleration_fault == fault) {
        active_acceleration_fault.clear();
        fault_deadline_active = false;
    }
    publishAccelerationFaultStatus("reset", fault, false, 0.0, "fault reset");
}

bool parseAccelerationFaultValue(
    const std::string& command,
    const std::string& prefix,
    double* value
) {
    if (command.compare(0, prefix.length(), prefix) != 0) {
        return false;
    }

    try {
        const double parsed = std::stod(command.substr(prefix.length()));
        if (!std::isfinite(parsed)) {
            ROS_WARN("Ignoring non-finite acceleration fault command: %s", command.c_str());
            return false;
        }
        *value = parsed;
        return true;
    } catch (const std::exception&) {
        ROS_WARN("Ignoring invalid acceleration fault command: %s", command.c_str());
        return false;
    }
}

void physicalFaultConfigCallback(const std_msgs::String::ConstPtr& message) {
    double value = 0.0;
    const std::string command = "PHYSICAL_FAULT_TIMEOUT_command";
    if (!parseAccelerationFaultValue(message->data, command + "=", &value)) {
        return;
    }

    if (!std::isfinite(value)) {
        publishAccelerationFaultStatus(
            "rejected", command, false, 0.0, "timeout must be a finite number"
        );
    } else if (fault_deadline_active) {
        publishAccelerationFaultStatus(
            "rejected", command, false, value, "cannot change timeout while an acceleration fault is active"
        );
    } else if (value <= 0.0 || value > max_fault_timeout_s) {
        publishAccelerationFaultStatus(
            "rejected", command, false, value, "timeout outside configured limits"
        );
    } else {
        fault_timeout_s = value;
        publishAccelerationFaultStatus(
            "configured", command, false, value, "physical fault timeout updated"
        );
    }
}

void vehicleCommandCallback(const std_msgs::String::ConstPtr& message) {
    double value = 0.0;

    if (parseAccelerationFaultValue(message->data, "ACCELOFFSET_command=", &value)) {
        const std::string fault = "ACCELOFFSET_command";
        if (value == 0.0) {
            acceleration_offset = 0.0;
            deactivateAccelerationFault(fault);
        } else if (!std::isfinite(value)) {
            publishAccelerationFaultStatus("rejected", fault, false, 0.0, "value must be finite");
        } else if (std::abs(value) > max_acceleration_offset) {
            ROS_WARN("Rejected acceleration offset outside physical limit: %f", value);
            publishAccelerationFaultStatus("rejected", fault, false, value, "outside physical limit");
        } else if (!canActivateAccelerationFault(fault)) {
            publishAccelerationFaultStatus("rejected", fault, false, value, "another acceleration fault is active");
        } else {
            acceleration_offset = value;
            activateAccelerationFault(fault, value);
        }
    } else if (message->data == "ACCELFREEZE_command=1") {
        const std::string fault = "ACCELFREEZE_command";
        if (!canActivateAccelerationFault(fault)) {
            publishAccelerationFaultStatus("rejected", fault, false, 1.0, "another acceleration fault is active");
        } else {
            acceleration_freeze_enabled = true;
            has_frozen_acceleration = false;
            activateAccelerationFault(fault, 1.0);
        }
    } else if (message->data == "ACCELFREEZE_command=0") {
        acceleration_freeze_enabled = false;
        has_frozen_acceleration = false;
        deactivateAccelerationFault("ACCELFREEZE_command");
    } else if (parseAccelerationFaultValue(message->data, "ACCELSAT_command=", &value)) {
        const std::string fault = "ACCELSAT_command";
        if (value == 0.0) {
            acceleration_saturation = 0.0;
            deactivateAccelerationFault(fault);
        } else if (!std::isfinite(value)) {
            publishAccelerationFaultStatus("rejected", fault, false, 0.0, "value must be finite");
        } else if (value < 0.0 || value > max_acceleration_saturation) {
            ROS_WARN("Rejected acceleration saturation outside physical limit: %f", value);
            publishAccelerationFaultStatus("rejected", fault, false, value, "outside physical limit");
        } else if (!canActivateAccelerationFault(fault)) {
            publishAccelerationFaultStatus("rejected", fault, false, value, "another acceleration fault is active");
        } else {
            acceleration_saturation = std::abs(value);
            activateAccelerationFault(fault, acceleration_saturation);
        }
    } else if (parseAccelerationFaultValue(message->data, "ACCELOSC_command=", &value)) {
        const std::string fault = "ACCELOSC_command";
        if (value == 0.0) {
            acceleration_oscillation = 0.0;
            deactivateAccelerationFault(fault);
        } else if (!std::isfinite(value)) {
            publishAccelerationFaultStatus("rejected", fault, false, 0.0, "value must be finite");
        } else if (value < 0.0 || value > max_acceleration_oscillation) {
            ROS_WARN("Rejected acceleration oscillation outside physical limit: %f", value);
            publishAccelerationFaultStatus("rejected", fault, false, value, "outside physical limit");
        } else if (!canActivateAccelerationFault(fault)) {
            publishAccelerationFaultStatus("rejected", fault, false, value, "another acceleration fault is active");
        } else {
            acceleration_oscillation = value;
            activateAccelerationFault(fault, value);
        }
    } else if (message->data == "ACCELRUNAWAY_command=1") {
        ROS_WARN("ACCELRUNAWAY is not applied on the physical actuator path");
        publishAccelerationFaultStatus(
            "rejected", "ACCELRUNAWAY_command", false, 1.0, "simulation only"
        );
    }
}

bool clearExpiredAccelerationFaults()
{
    if (!fault_deadline_active || ros::WallTime::now() < fault_deadline) {
        return false;
    }

    const std::string expired_fault = active_acceleration_fault;
    acceleration_offset = 0.0;
    acceleration_saturation = 0.0;
    acceleration_oscillation = 0.0;
    acceleration_freeze_enabled = false;
    has_frozen_acceleration = false;
    fault_deadline_active = false;
    active_acceleration_fault.clear();
    ROS_ERROR("Physical acceleration fault timeout expired; reset to normal command.");
    publishAccelerationFaultStatus(
        "timeout", expired_fault, false, 0.0, "hard timeout expired"
    );
    return true;
}
// ===== End AD-EYE fault-injection change =====

class VehicleController {
public:
    VehicleController(ros::NodeHandle *nh, double P = 0.1, double I = 0.0, double D = 0.0)
        : P_(P), I_(I), D_(D), accelPIDController(P_, I_, D_, std::bind(&VehicleController::pidSource, this), std::bind(&VehicleController::pidOutput, this, std::placeholders::_1)) {

        linear_speed_target_ = 0.0,
        angular_speed_target_ = 0.0;
        linear_speed_current_ = 0.0;
        angular_speed_current_ = 0.0;

        accelPIDController.registerTimeFunction(&VehicleController::pidTimeFunction);
        pub_acceleration = nh->advertise<std_msgs::Float64>(topic_acceleration_request, 1);
        pub_steering_angle = nh->advertise<std_msgs::Float64>(topic_steering_request, 1);
    }

    void velocityRequestedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        ROS_DEBUG("Desired TwistS: Long. Vel. = %f, Rot. Spd. = %f", msg->twist.linear.x, msg->twist.angular.z);
        linear_speed_target_ = msg->twist.linear.x;
        angular_speed_target_ = msg->twist.angular.z;

        accelPIDController.setTarget(linear_speed_target_);
    }

    void velocityCurrentCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
        ROS_DEBUG("Current velocity: Long. Vel. = %f, Rot. Spd. = %f", msg->twist.linear.x, msg->twist.angular.z);
        linear_speed_current_ = msg->twist.linear.x;
        angular_speed_current_ = msg->twist.angular.z;
    }

    void updatePID() {
        accelPIDController.tick();
    }

    void updateSteering() {
        double steering_angle_command = steering(angular_speed_target_);
        sendSteeringAngle(steering_angle_command);
    }

    void accelerationFaultWatchdogCallback(const ros::WallTimerEvent&) {
        if (clearExpiredAccelerationFaults() && has_last_unfaulted_acceleration) {
            std_msgs::Float64 msg;
            msg.data = last_unfaulted_acceleration;
            pub_acceleration.publish(msg);
        } else if (fault_deadline_active) {
            double active_value = 1.0;
            if (active_acceleration_fault == "ACCELOFFSET_command") {
                active_value = acceleration_offset;
            } else if (active_acceleration_fault == "ACCELSAT_command") {
                active_value = acceleration_saturation;
            } else if (active_acceleration_fault == "ACCELOSC_command") {
                active_value = acceleration_oscillation;
            }
            publishAccelerationFaultStatus(
                "active", active_acceleration_fault, true, active_value, "fault active"
            );
        }
    }

private:
    void sendAcceleration(double req) {
        // ===== AD-EYE fault-injection change: apply GUI fault state =====
        clearExpiredAccelerationFaults();
        last_unfaulted_acceleration = req;
        has_last_unfaulted_acceleration = true;
        double faulted_acceleration = req;

        if (acceleration_freeze_enabled) {
            if (!has_frozen_acceleration) {
                frozen_acceleration = faulted_acceleration;
                has_frozen_acceleration = true;
            }
            faulted_acceleration = frozen_acceleration;
        }

        faulted_acceleration += acceleration_offset;

        if (acceleration_saturation > 0.0 && faulted_acceleration > acceleration_saturation) {
            faulted_acceleration = acceleration_saturation;
        }

        if (acceleration_oscillation != 0.0) {
            faulted_acceleration += acceleration_oscillation * std::sin(ros::Time::now().toSec());
        }

        std_msgs::Float64 msg;
        // Legacy, pre-fault behaviour: msg.data = req;
        msg.data = faulted_acceleration;
        // ===== End AD-EYE fault-injection change =====
        pub_acceleration.publish(msg);
    }

    void sendSteeringAngle(double req) {
        std_msgs::Float64 msg;
        msg.data = req;
        pub_steering_angle.publish(msg);
    }

    double steering(double angular_vel) {
        using namespace std;

        static const double WHEELBASE = 2.984;
        static const double MAX_STEERING_ANGLE = 0.85;
        static const double MAX_STEERING_TAN = tan(MAX_STEERING_ANGLE);

        double steer_ag_req = 0.0;
        if (linear_speed_current_ != 0.0) {
            steer_ag_req = atan(max(min(WHEELBASE * angular_vel / linear_speed_current_, MAX_STEERING_TAN), -MAX_STEERING_TAN));
        }
        return steer_ag_req;
    }

    double pidSource() {
        return linear_speed_current_;
    }

    void pidOutput(double acceleration) {
        if (! std::isnan(acceleration)) {
            sendAcceleration(acceleration);
        }
    }

    static unsigned long pidTimeFunction() {
        return static_cast<unsigned long>(ros::Time::now().toSec() * 1000);
    }

private:
    double linear_speed_target_;
    double linear_speed_current_;
    double angular_speed_target_;
    double angular_speed_current_;

    double P_, I_, D_;

    ros::Publisher pub_acceleration;
    ros::Publisher pub_steering_angle;
    PIDController<double> accelPIDController;
};

int main(int argc, char** argv) {

    ros::init(argc, argv, "vehicle_controller");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    std::string topic_velocity_current;

    private_nh.getParam("topic_velocity_current", topic_velocity_current);
    private_nh.getParam("topic_acceleration_request", topic_acceleration_request);
    private_nh.getParam("topic_steering_request", topic_steering_request);
    private_nh.param("max_acceleration_offset", max_acceleration_offset, 0.2);
    private_nh.param("max_acceleration_saturation", max_acceleration_saturation, 0.5);
    private_nh.param("max_acceleration_oscillation", max_acceleration_oscillation, 0.1);
    private_nh.param("fault_timeout_s", fault_timeout_s, 5.0);
    private_nh.param("max_fault_timeout_s", max_fault_timeout_s, 300.0);
    if (max_fault_timeout_s <= 0.0) {
        ROS_WARN("Invalid maximum acceleration fault timeout; using 300 seconds.");
        max_fault_timeout_s = 300.0;
    }
    if (fault_timeout_s <= 0.0) {
        ROS_WARN("Invalid acceleration fault timeout; using 5 seconds.");
        fault_timeout_s = 5.0;
    } else if (fault_timeout_s > max_fault_timeout_s) {
        ROS_WARN("Acceleration fault timeout exceeds maximum; using maximum.");
        fault_timeout_s = max_fault_timeout_s;
    }

    double P;
    double I;
    double D;
    if (argc > 3 )
    {
        P = atof(argv[1]);
        I = atof(argv[2]);
        D = atof(argv[3]);
    }
    else {
        P = 0.1;
        I = 0.0;
        D = 0.0;
    }
    VehicleController ttv(&nh,P,I,D);
    acceleration_fault_status_pub = nh.advertise<std_msgs::String>(
        "/adeye/physical_fault_status", 10
    );
    ros::Subscriber sub_velocity_requested = nh.subscribe("TwistS", 2, &VehicleController::velocityRequestedCallback, &ttv);
    ros::Subscriber sub_velocity_current = nh.subscribe(topic_velocity_current, 2, &VehicleController::velocityCurrentCallback, &ttv);
    // AD-EYE fault-injection change: GUI fault-control subscription.
    ros::Subscriber sub_vehicle_commands = nh.subscribe("/vehicle_commands", 10, vehicleCommandCallback);
    ros::Subscriber sub_physical_fault_config = nh.subscribe(
        "/adeye/physical_fault_config", 10, physicalFaultConfigCallback
    );
    ros::WallTimer acceleration_fault_watchdog = nh.createWallTimer(
        ros::WallDuration(0.1),
        &VehicleController::accelerationFaultWatchdogCallback,
        &ttv
    );
    ros::Rate r(10);
    while (ros::ok()) {
        ros::spinOnce();
        ttv.updatePID();
        ttv.updateSteering();
        r.sleep();
    }
    return 0;
}
