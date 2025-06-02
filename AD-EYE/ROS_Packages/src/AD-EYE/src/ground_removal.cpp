/*
 * ground_removal.cpp
 * Advanced LiDAR-based ground removal node
 * Uses Progressive Morphological Filter and RANSAC for robust ground detection
 * Subscribes to voxelized point cloud data and publishes ground and non-ground points
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/common/common.h>

#include <vector>
#include <string>

class GroundRemovalNode {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscribers and Publishers
    ros::Subscriber cloud_sub_;
    ros::Publisher ground_pub_;
    ros::Publisher non_ground_pub_;
    ros::Publisher marker_pub_;

    // Parameters
    std::string input_topic_;
    std::string output_frame_;
    bool enable_pmf_filter_;
    bool enable_ransac_fallback_;
    bool enable_preprocessing_;

    // Progressive Morphological Filter parameters
    int pmf_max_window_size_;
    float pmf_slope_;
    float pmf_initial_distance_;
    float pmf_max_distance_;
    float pmf_cell_size_;

    // RANSAC parameters (fallback)
    double ransac_distance_threshold_;
    int ransac_max_iterations_;

    // ROI parameters for preprocessing
    double roi_min_x_, roi_max_x_;
    double roi_min_y_, roi_max_y_;
    double roi_min_z_, roi_max_z_;

    // Statistics
    int processed_frames_;
    double total_processing_time_;

public:
    GroundRemovalNode() : private_nh_("~"), processed_frames_(0), total_processing_time_(0.0) {
        initializeParameters();
        initializePublishersSubscribers();

        ROS_INFO("Ground Removal Node initialized");
        ROS_INFO("Input topic: %s", input_topic_.c_str());
        ROS_INFO("PMF enabled: %s, RANSAC fallback: %s",
                 enable_pmf_filter_ ? "true" : "false",
                 enable_ransac_fallback_ ? "true" : "false");
        ROS_INFO("PMF parameters - Window: %d, Slope: %.2f, Initial: %.2f, Max: %.2f",
                 pmf_max_window_size_, pmf_slope_, pmf_initial_distance_, pmf_max_distance_);
    }

    void initializeParameters() {
        // Topic parameters
        private_nh_.param<std::string>("input_topic", input_topic_, "/filtered_points");
        private_nh_.param<std::string>("output_frame", output_frame_, "base_link");

        // Algorithm selection
        private_nh_.param<bool>("enable_pmf_filter", enable_pmf_filter_, true);
        private_nh_.param<bool>("enable_ransac_fallback", enable_ransac_fallback_, true);
        private_nh_.param<bool>("enable_preprocessing", enable_preprocessing_, true);

        // Progressive Morphological Filter parameters
        private_nh_.param<int>("pmf_max_window_size", pmf_max_window_size_, 20);
        private_nh_.param<float>("pmf_slope", pmf_slope_, 1.0f);
        private_nh_.param<float>("pmf_initial_distance", pmf_initial_distance_, 0.5f);
        private_nh_.param<float>("pmf_max_distance", pmf_max_distance_, 3.0f);
        private_nh_.param<float>("pmf_cell_size", pmf_cell_size_, 1.0f);

        // RANSAC parameters
        private_nh_.param<double>("ransac_distance_threshold", ransac_distance_threshold_, 0.2);
        private_nh_.param<int>("ransac_max_iterations", ransac_max_iterations_, 100);

        // ROI parameters
        private_nh_.param<double>("roi_min_x", roi_min_x_, -50.0);
        private_nh_.param<double>("roi_max_x", roi_max_x_, 50.0);
        private_nh_.param<double>("roi_min_y", roi_min_y_, -50.0);
        private_nh_.param<double>("roi_max_y", roi_max_y_, 50.0);
        private_nh_.param<double>("roi_min_z", roi_min_z_, -3.0);
        private_nh_.param<double>("roi_max_z", roi_max_z_, 5.0);
    }

    void initializePublishersSubscribers() {
        cloud_sub_ = nh_.subscribe(input_topic_, 1, &GroundRemovalNode::cloudCallback, this);

        ground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ground_removal/ground_points", 1);
        non_ground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/ground_removal/non_ground_points", 1);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/ground_removal/markers", 1);
    }

    void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input_cloud) {
        auto start_time = ros::Time::now();

        // Convert to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input_cloud, *cloud);

        if (cloud->empty()) {
            ROS_WARN("Received empty point cloud");
            return;
        }

        // Preprocess the cloud if enabled
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = cloud;
        if (enable_preprocessing_) {
            filtered_cloud = preprocessCloud(cloud);
            if (filtered_cloud->empty()) {
                ROS_WARN("Empty cloud after preprocessing");
                return;
            }
        }

        // Perform ground removal
        std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> result;

        if (enable_pmf_filter_) {
            result = removeGroundPMF(filtered_cloud);

            // Check if PMF failed and use RANSAC fallback
            if (enable_ransac_fallback_ && result.first->empty() && result.second->empty()) {
                ROS_WARN("PMF failed, using RANSAC fallback");
                result = removeGroundRANSAC(filtered_cloud);
            }
        } else {
            result = removeGroundRANSAC(filtered_cloud);
        }

        // Publish results
        publishResults(result.first, result.second, input_cloud->header);

        // Update statistics
        auto processing_time = (ros::Time::now() - start_time).toSec();
        total_processing_time_ += processing_time;
        processed_frames_++;

        double avg_time = total_processing_time_ / processed_frames_;
        ROS_INFO("Ground removal completed: Ground=%zu, Non-ground=%zu points. Time=%.3fms (avg=%.3fms)",
                 result.first->size(), result.second->size(),
                 processing_time * 1000.0, avg_time * 1000.0);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // ROI filtering to remove distant points
        pcl::PassThrough<pcl::PointXYZ> pass;

        // X direction
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(roi_min_x_, roi_max_x_);
        pass.filter(*filtered_cloud);

        // Y direction
        pass.setInputCloud(filtered_cloud);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(roi_min_y_, roi_max_y_);
        pass.filter(*filtered_cloud);

        // Z direction (most important for ground removal)
        pass.setInputCloud(filtered_cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(roi_min_z_, roi_max_z_);
        pass.filter(*filtered_cloud);

        return filtered_cloud;
    }

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr>
    removeGroundPMF(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        try {
            // Create Progressive Morphological Filter
            pcl::PointIndicesPtr ground_indices(new pcl::PointIndices);
            pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ> pmf;

            pmf.setInputCloud(cloud);
            pmf.setMaxWindowSize(pmf_max_window_size_);
            pmf.setSlope(pmf_slope_);
            pmf.setInitialDistance(pmf_initial_distance_);
            pmf.setMaxDistance(pmf_max_distance_);
            pmf.setCellSize(pmf_cell_size_);

            // Extract ground indices
            pmf.extract(ground_indices->indices);

            if (ground_indices->indices.empty()) {
                ROS_WARN("PMF found no ground points");
                return std::make_pair(ground_cloud, non_ground_cloud);
            }

            // Extract ground and non-ground points using ExtractIndices
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(ground_indices);

            // Extract ground points
            extract.setNegative(false);
            extract.filter(*ground_cloud);

            // Extract non-ground points
            extract.setNegative(true);
            extract.filter(*non_ground_cloud);

        } catch (const std::exception& e) {
            ROS_ERROR("PMF ground removal failed: %s", e.what());
            return std::make_pair(ground_cloud, non_ground_cloud);
        }

        return std::make_pair(ground_cloud, non_ground_cloud);
    }

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr>
    removeGroundRANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        try {
            // RANSAC plane segmentation for ground detection
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(ransac_max_iterations_);
            seg.setDistanceThreshold(ransac_distance_threshold_);

            seg.setInputCloud(cloud);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.empty()) {
                ROS_WARN("RANSAC could not estimate ground plane");
                non_ground_cloud = cloud; // Treat all points as non-ground
                return std::make_pair(ground_cloud, non_ground_cloud);
            }

            // Extract ground and non-ground points
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);

            // Extract ground points
            extract.setNegative(false);
            extract.filter(*ground_cloud);

            // Extract non-ground points
            extract.setNegative(true);
            extract.filter(*non_ground_cloud);

            // Validate plane orientation (should be roughly horizontal)
            if (coefficients->values.size() >= 4) {
                double normal_z = std::abs(coefficients->values[2]);
                if (normal_z < 0.7) { // Normal should point mostly upward
                    ROS_WARN("Detected plane is not horizontal (normal_z=%.2f), may not be ground", normal_z);
                }
            }

        } catch (const std::exception& e) {
            ROS_ERROR("RANSAC ground removal failed: %s", e.what());
            return std::make_pair(ground_cloud, non_ground_cloud);
        }

        return std::make_pair(ground_cloud, non_ground_cloud);
    }

    void publishResults(pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud,
                       const std_msgs::Header& header) {

        // Publish ground points
        if (!ground_cloud->empty()) {
            sensor_msgs::PointCloud2 ground_msg;
            pcl::toROSMsg(*ground_cloud, ground_msg);
            ground_msg.header = header;
            ground_msg.header.frame_id = output_frame_;
            ground_pub_.publish(ground_msg);
        }

        // Publish non-ground points
        if (!non_ground_cloud->empty()) {
            sensor_msgs::PointCloud2 non_ground_msg;
            pcl::toROSMsg(*non_ground_cloud, non_ground_msg);
            non_ground_msg.header = header;
            non_ground_msg.header.frame_id = output_frame_;
            non_ground_pub_.publish(non_ground_msg);
        }

        // Publish visualization markers
        publishVisualizationMarkers(ground_cloud, non_ground_cloud, header);
    }

    void publishVisualizationMarkers(pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud,
                                   pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground_cloud,
                                   const std_msgs::Header& header) {

        visualization_msgs::MarkerArray marker_array;

        // Clear previous markers
        visualization_msgs::Marker clear_marker;
        clear_marker.header = header;
        clear_marker.header.frame_id = output_frame_;
        clear_marker.ns = "ground_removal";
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        // Statistics text marker
        visualization_msgs::Marker stats_marker;
        stats_marker.header = header;
        stats_marker.header.frame_id = output_frame_;
        stats_marker.ns = "ground_removal_stats";
        stats_marker.id = 0;
        stats_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        stats_marker.action = visualization_msgs::Marker::ADD;

        stats_marker.pose.position.x = 0.0;
        stats_marker.pose.position.y = 0.0;
        stats_marker.pose.position.z = 3.0;
        stats_marker.pose.orientation.w = 1.0;

        stats_marker.scale.z = 1.0;
        stats_marker.color.r = 1.0;
        stats_marker.color.g = 1.0;
        stats_marker.color.b = 1.0;
        stats_marker.color.a = 1.0;

        double total_points = ground_cloud->size() + non_ground_cloud->size();
        double ground_percentage = total_points > 0 ? (ground_cloud->size() * 100.0 / total_points) : 0.0;

        stats_marker.text = "Ground: " + std::to_string(ground_cloud->size()) +
                           "\nNon-Ground: " + std::to_string(non_ground_cloud->size()) +
                           "\nGround %: " + std::to_string((int)ground_percentage) + "%";

        stats_marker.lifetime = ros::Duration(1.0);
        marker_array.markers.push_back(stats_marker);

        marker_pub_.publish(marker_array);
    }

    ~GroundRemovalNode() {
        if (processed_frames_ > 0) {
            double avg_time = total_processing_time_ / processed_frames_;
            ROS_INFO("Ground Removal Node shutting down. Processed %d frames, average time: %.3fms",
                     processed_frames_, avg_time * 1000.0);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ground_removal");
    GroundRemovalNode ground_removal;
    ros::spin();
    return 0;
}