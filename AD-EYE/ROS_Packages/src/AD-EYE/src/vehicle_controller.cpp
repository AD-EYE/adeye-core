#include <algorithm>
#include <cmath>
#include <exception>
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
ros::WallTime fault_deadline;
bool fault_deadline_active = false;

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

void vehicleCommandCallback(const std_msgs::String::ConstPtr& message) {
    double value = 0.0;

    if (parseAccelerationFaultValue(message->data, "ACCELOFFSET_command=", &value)) {
        if (std::abs(value) <= max_acceleration_offset) {
            acceleration_offset = value;
        } else {
            ROS_WARN("Rejected acceleration offset outside physical limit: %f", value);
        }
    } else if (message->data == "ACCELFREEZE_command=1") {
        acceleration_freeze_enabled = true;
        has_frozen_acceleration = false;
    } else if (message->data == "ACCELFREEZE_command=0") {
        acceleration_freeze_enabled = false;
        has_frozen_acceleration = false;
    } else if (parseAccelerationFaultValue(message->data, "ACCELSAT_command=", &value)) {
        if (std::abs(value) <= max_acceleration_saturation) {
            acceleration_saturation = std::abs(value);
        } else {
            ROS_WARN("Rejected acceleration saturation outside physical limit: %f", value);
        }
    } else if (parseAccelerationFaultValue(message->data, "ACCELOSC_command=", &value)) {
        if (std::abs(value) <= max_acceleration_oscillation) {
            acceleration_oscillation = value;
        } else {
            ROS_WARN("Rejected acceleration oscillation outside physical limit: %f", value);
        }
    } else if (message->data == "ACCELRUNAWAY_command=1") {
        ROS_WARN("ACCELRUNAWAY is not applied on the physical actuator path");
    }

    if (acceleration_offset != 0.0 || acceleration_saturation != 0.0 ||
        acceleration_oscillation != 0.0 || acceleration_freeze_enabled) {
        fault_deadline = ros::WallTime::now() + ros::WallDuration(fault_timeout_s);
        fault_deadline_active = true;
    } else {
        fault_deadline_active = false;
    }
}

void clearExpiredAccelerationFaults()
{
    if (!fault_deadline_active || ros::WallTime::now() < fault_deadline) {
        return;
    }

    acceleration_offset = 0.0;
    acceleration_saturation = 0.0;
    acceleration_oscillation = 0.0;
    acceleration_freeze_enabled = false;
    has_frozen_acceleration = false;
    fault_deadline_active = false;
    ROS_ERROR("Physical acceleration fault timeout expired; reset to normal command.");
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

private:
    void sendAcceleration(double req) {
        // ===== AD-EYE fault-injection change: apply GUI fault state =====
        clearExpiredAccelerationFaults();
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
    if (fault_timeout_s <= 0.0) {
        ROS_WARN("Invalid acceleration fault timeout; using 5 seconds.");
        fault_timeout_s = 5.0;
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
    ros::Subscriber sub_velocity_requested = nh.subscribe("TwistS", 2, &VehicleController::velocityRequestedCallback, &ttv);
    ros::Subscriber sub_velocity_current = nh.subscribe(topic_velocity_current, 2, &VehicleController::velocityCurrentCallback, &ttv);
    // AD-EYE fault-injection change: GUI fault-control subscription.
    ros::Subscriber sub_vehicle_commands = nh.subscribe("/vehicle_commands", 10, vehicleCommandCallback);
    ros::Rate r(10);
    while (ros::ok()) {
        ros::spinOnce();
        ttv.updatePID();
        ttv.updateSteering();
        r.sleep();
    }
    return 0;
}
