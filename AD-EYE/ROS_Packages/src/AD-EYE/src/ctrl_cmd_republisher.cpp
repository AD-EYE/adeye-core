#include <ros/ros.h>
#include <autoware_msgs/ControlCommandStamped.h>
#include <std_msgs/Float64.h>

ros::Publisher pub;

void ctrlCmdCallback(const autoware_msgs::ControlCommandStamped::ConstPtr& control_cmd)
{
    std_msgs::Float64 steering_cmd;
    float command = control_cmd->cmd.steering_angle;

    if (command < -0.85)
        command = -0.85;
    if (command > 0.85)
        command = 0.85;

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