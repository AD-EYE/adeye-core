#ifndef ADEYE_RECOVERY_GATE_H
#define ADEYE_RECOVERY_GATE_H
#include <functional>
#include <memory>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

// Runs only when the user requests a reset. Does not stop the car because a command is old.
class RecoveryGate {
public:
    RecoveryGate(ros::NodeHandle& nh, std::function<void()> clear, bool new_goal = false)
        : clear_(clear), new_goal_(new_goal) {
        nh.param("/adeye/enable_manual_recovery", enabled_, false);
        if (enabled_) nh.getParam("/adeye/vehicle_recovery_token", token_);
        held_ = !token_.empty();
        ack_ = nh.advertise<std_msgs::String>("/adeye/vehicle_recovery_ack", 10, true);
        sub_ = nh.subscribe("/adeye/vehicle_recovery_control", 10, &RecoveryGate::callback, this);
        goal_sub_ = nh.subscribe("/move_base_simple/goal", 1, &RecoveryGate::goal, this);
    }
    bool blocked() const { return held_ || waiting_goal_; }
private:
    void goal(const geometry_msgs::PoseStamped::ConstPtr&) {
        if (!held_ && waiting_goal_) { clear_(); waiting_goal_ = false; }
    }
    void callback(const std_msgs::String::ConstPtr& message) {
        if (!enabled_) return;
        const auto split = message->data.find(':');
        if (split == std::string::npos) return;
        const std::string action = message->data.substr(0, split), token = message->data.substr(split + 1);
        if (token.empty() || token.find_first_not_of("0123456789abcdef") != std::string::npos) return;
        std::string phase;
        if (action == "PREPARE") {
            held_ = true;
            if (token != prepared_) { token_ = token; clear_(); prepared_ = token; }
            phase = "prepared";
        } else if (action == "RELEASE" && token == prepared_) {
            if (held_) { clear_(); waiting_goal_ = new_goal_; held_ = false; }
            phase = "released";
        } else return;
        std_msgs::String ack;
        ack.data = "{\"component\":\"" + ros::this_node::getName() + "\",\"token\":\"" + token + "\",\"phase\":\"" + phase + "\"}";
        ack_.publish(ack);
    }
    bool enabled_ = false, held_ = false, waiting_goal_ = false, new_goal_;
    std::string token_, prepared_;
    std::function<void()> clear_;
    ros::Publisher ack_;
    ros::Subscriber sub_, goal_sub_;
};
#endif
