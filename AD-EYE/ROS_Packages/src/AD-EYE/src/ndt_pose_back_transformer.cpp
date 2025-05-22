#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <ros/console.h>

#include <geometry_msgs/PoseStamped.h>

class NDTBackTransform
{
    ros::NodeHandle& nh_;
    ros::Subscriber sub_ndt_pose_;
    ros::Publisher pub_current_pose_;

    ros::Rate rate_;

    tf::TransformListener tf_listener_;

    public:

    void NDTPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        geometry_msgs::PoseStamped transformed_pose_;

        try
        {
            tf_listener_.transformPose("/map", *msg, transformed_pose_);
            pub_current_pose_.publish(transformed_pose_);
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("Transforming NDT Pose back failed");
        }
    }

    NDTBackTransform(ros::NodeHandle& nh) : nh_(nh), rate_(10)
    {
        // Initialize node, publishers and subscribers
        sub_ndt_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>("/ndt_pose", 1, &NDTBackTransform::NDTPoseCallback, this);
        pub_current_pose_ = nh_.advertise<geometry_msgs::PoseStamped>("/current_pose", 1, true);
    }

    void run()
    {
        while (nh_.ok())
        {
            ros::spinOnce();
            rate_.sleep();
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "NDT_Pose_Back_Transform");
    ros::NodeHandle nh;

    NDTBackTransform ndt_transformer(nh);
    ndt_transformer.run();
    
    return 0;
}