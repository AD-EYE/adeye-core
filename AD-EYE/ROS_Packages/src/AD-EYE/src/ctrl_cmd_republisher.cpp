#include <ros/ros.h>
#include <autoware_msgs/ControlCommandStamped.h>
#include <std_msgs/Float64.h>

ros::Publisher pub;

const double g_MAX_STEERING_ANGLE {0.85};

void ctrlCmdCallback(const autoware_msgs::ControlCommandStamped::ConstPtr& control_cmd)
{
    std_msgs::Float64 steering_cmd;
    float command = control_cmd->cmd.steering_angle;

    if (command < -g_MAX_STEERING_ANGLE)
        command = -g_MAX_STEERING_ANGLE;
    if (command > g_MAX_STEERING_ANGLE)
        command = g_MAX_STEERING_ANGLE;

    steering_cmd.data = command;

    pub.publish(steering_cmd);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ctrl_cmd_republisher");
    ros::NodeHandle nh;

    ros::Subscriber sub_ctrl_cmd = nh.subscribe<autoware_msgs::ControlCommandStamped>("/ctrl_cmd", 1, ctrlCmdCallback);

    pub = nh.advertise<std_msgs::Float64>("/steering_requested_phy", 1);

    ros::spin();

    return 0;
}