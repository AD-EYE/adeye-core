#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

//////////////////////////////////////////////////////////
//////////////////// Class definition ////////////////////

class LocalizationFusion {
public:
    LocalizationFusion() : nh_("~"), has_lidar_data_(false), has_gps_data_(false) {
        // Subscribing to LIDAR and GPS topics
        lidar_sub_ = nh_.subscribe("/ndt_pose", 10, &LocalizationFusion::lidarCallback, this);
        gps_sub_ = nh_.subscribe("/gnss_pose", 10, &LocalizationFusion::gpsCallback, this);
        //fix_sub_ = nh_.subscribe("/fix", 10, &LocalizationFusion::fixCallback, this);

        // Advertising the merged topic
        merged_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/merged_pose", 10);

        // Initializing the variables
        current_lidar_pose_.pose.orientation.w = 1.0; // Default orientation
    }

    //////////////////// LIDAR callback function ////////////////////
    void lidarCallback(const geometry_msgs::PoseStamped::ConstPtr& lidar_msg) {
        ROS_INFO("Received LIDAR data");
        current_lidar_pose_.pose.position = lidar_msg->pose.position;
        current_lidar_pose_.pose.orientation = lidar_msg->pose.orientation;
        current_lidar_pose_.header = lidar_msg->header;
        has_lidar_data_ = true;
        mergeData();
    }

    //////////////////// GPS callback function ////////////////////
    void gpsCallback(const geometry_msgs::PoseStamped::ConstPtr& gps_msg) {
        current_gps_pose_ = *gps_msg;
        ROS_INFO("Received GPS data");
        has_gps_data_ = true;
        mergeData();
    }

    //////////////////// Fix callback function ////////////////////
    void fixCallback(const sensor_msgs::NavSatFix::ConstPtr& fix_msg) {
        current_fix_pose_ = *fix_msg;
        ROS_INFO("Received Fix data");
        has_fix_data_ = true;
        mergeData();
    }

    //////////////////// Merging data function ////////////////////
    void mergeData() {
    geometry_msgs::PoseStamped merged_pose;
    merged_pose.header.stamp = ros::Time::now();

    if (!has_lidar_data_) {
        // No LiDAR data available
        ROS_WARN("No LiDAR data received!");
        return;
    }

    if (!has_gps_data_) {
        // Use only LiDAR data if no GPS data is available
        ROS_WARN("No GPS data received!");
        merged_pose.header.frame_id = current_lidar_pose_.header.frame_id;
        merged_pose.pose = current_lidar_pose_.pose;
    } 
    
    else {
        // Use both types of data if available
        merged_pose.header.frame_id = current_lidar_pose_.header.frame_id;
        double gps_weight = 1.0; // / (current_gps_pose_.pose.covariance[0] + current_gps_pose_.pose.covariance[7] + current_gps_pose_.pose.covariance[14]);
        double lidar_weight = 1.0;
        // Merge positions
        merged_pose.pose.position.x = (gps_weight * current_gps_pose_.pose.position.x + lidar_weight * current_lidar_pose_.pose.position.x) / (gps_weight + lidar_weight);
        merged_pose.pose.position.y = (gps_weight * current_gps_pose_.pose.position.y + lidar_weight * current_lidar_pose_.pose.position.y) / (gps_weight + lidar_weight);
        merged_pose.pose.position.z = (gps_weight * current_gps_pose_.pose.position.z + lidar_weight * current_lidar_pose_.pose.position.z) / (gps_weight + lidar_weight);
        // Use LiDAR orientation
        merged_pose.pose.orientation = current_lidar_pose_.pose.orientation;
    }

    // Publication of merged pose
    merged_pub_.publish(merged_pose);
    publishBaseLinkTransform(merged_pose);
}

private:
    ros::NodeHandle nh_;
    ros::Subscriber lidar_sub_;
    ros::Subscriber gps_sub_;
    //ros::Subscriber fix_sub_;
    ros::Publisher merged_pub_;

    geometry_msgs::PoseStamped current_lidar_pose_;
    geometry_msgs::PoseStamped current_gps_pose_;

    bool has_lidar_data_;
    bool has_gps_data_;
    //bool has_fix_data_;

    void publishBaseLinkTransform(const geometry_msgs::PoseStamped& merged_pose) {
        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(
            merged_pose.pose.position.x,
            merged_pose.pose.position.y,
            merged_pose.pose.position.z
        ));
        tf::Quaternion q(
            merged_pose.pose.orientation.x,
            merged_pose.pose.orientation.y,
            merged_pose.pose.orientation.z,
            merged_pose.pose.orientation.w
        );
        transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(transform, merged_pose.header.stamp, "map", "base_link"));
    }
};

///////////////////////////////////////////////////////
//////////////////// Main function ////////////////////

int main(int argc, char** argv) {
    ros::init(argc, argv, "LocalizationFusion");
    LocalizationFusion LocalizationFusion;
    ros::spin();
    return 0;
}
