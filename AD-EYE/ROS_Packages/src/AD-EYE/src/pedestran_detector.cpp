/*
 * simple_pedestrian_detector.cpp
 * Simple LiDAR-based pedestrian detection node
 * Detects and tracks pedestrians using point cloud clustering and geometric constraints
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/BoundingBox.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#include <vector>
#include <string>
#include <algorithm>

struct PedestrianBox {
    int id;
    pcl::PointXYZ center;
    pcl::PointXYZ min_point;
    pcl::PointXYZ max_point;
    float width, height, depth;
    float confidence;
    int point_count;

    PedestrianBox() : id(0), confidence(0.0f), point_count(0) {}
};

class SimplePedestrianDetector {
private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    // Subscribers and Publishers
    ros::Subscriber cloud_sub_;
    ros::Publisher bbox_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher pedestrian_cloud_pub_;
    ros::Publisher ground_cloud_pub_;

    // Parameters
    std::string input_topic_;
    std::string output_frame_;
    bool enable_ground_removal_;

    // Detection parameters
    double voxel_leaf_size_;
    double ground_threshold_;
    double cluster_tolerance_;
    int min_cluster_size_;
    int max_cluster_size_;

    // Pedestrian constraints
    double min_height_;
    double max_height_;
    double min_width_;
    double max_width_;
    double max_depth_;
    double roi_min_x_, roi_max_x_;
    double roi_min_y_, roi_max_y_;
    double roi_min_z_, roi_max_z_;

    // Tracking
    std::vector<PedestrianBox> previous_pedestrians_;
    int next_id_;

public:
    SimplePedestrianDetector() : private_nh_("~"), next_id_(1) {
        initializeParameters();
        initializePublishersSubscribers();

        ROS_INFO("Simple Pedestrian Detector initialized");
        ROS_INFO("Listening to: %s", input_topic_.c_str());
        ROS_INFO("Detection constraints - Height:[%.2f-%.2f] Width:[%.2f-%.2f] Depth:[0-%.2f]",
                 min_height_, max_height_, min_width_, max_width_, max_depth_);
    }

    void initializeParameters() {
        // Topic parameters
        private_nh_.param<std::string>("input_topic", input_topic_, "/filtered_points");
        private_nh_.param<std::string>("output_frame", output_frame_, "base_link");
        private_nh_.param<bool>("enable_ground_removal", enable_ground_removal_, true);

        // Processing parameters
        private_nh_.param<double>("voxel_leaf_size", voxel_leaf_size_, 0.1);
        private_nh_.param<double>("ground_threshold", ground_threshold_, 0.2);
        private_nh_.param<double>("cluster_tolerance", cluster_tolerance_, 0.5);
        private_nh_.param<int>("min_cluster_size", min_cluster_size_, 20);
        private_nh_.param<int>("max_cluster_size", max_cluster_size_, 400);

        // Pedestrian constraints
        private_nh_.param<double>("min_height", min_height_, 1.2);
        private_nh_.param<double>("max_height", max_height_, 2.1);
        private_nh_.param<double>("min_width", min_width_, 0.3);
        private_nh_.param<double>("max_width", max_width_, 0.8);
        private_nh_.param<double>("max_depth", max_depth_, 0.7);

        // ROI parameters
        private_nh_.param<double>("roi_min_x", roi_min_x_, -2.0);
        private_nh_.param<double>("roi_max_x", roi_max_x_, 30.0);
        private_nh_.param<double>("roi_min_y", roi_min_y_, -15.0);
        private_nh_.param<double>("roi_max_y", roi_max_y_, 15.0);
        private_nh_.param<double>("roi_min_z", roi_min_z_, -0.5);
        private_nh_.param<double>("roi_max_z", roi_max_z_, 2.5);
    }

    void initializePublishersSubscribers() {
        cloud_sub_ = nh_.subscribe(input_topic_, 1, &SimplePedestrianDetector::cloudCallback, this);

        bbox_pub_ = nh_.advertise<jsk_recognition_msgs::BoundingBoxArray>("/pedestrian_detection/bounding_boxes", 1);
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/pedestrian_detection/markers", 1);
        pedestrian_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pedestrian_detection/pedestrian_points", 1);
        ground_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/pedestrian_detection/ground_points", 1);
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

        // Process the point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud = preprocessCloud(cloud);

        std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmented_clouds;

        if (enable_ground_removal_) {
            segmented_clouds = removeGround(filtered_cloud);
        } else {
            segmented_clouds.first = filtered_cloud;
            segmented_clouds.second = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
        }

        // Cluster objects
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters = clusterObjects(segmented_clouds.first);

        // Detect pedestrians
        std::vector<PedestrianBox> pedestrians = detectPedestrians(clusters);

        // Track pedestrians
        trackPedestrians(pedestrians);

        // Publish results
        publishResults(pedestrians, segmented_clouds, input_cloud->header);

        auto processing_time = (ros::Time::now() - start_time).toSec();
        ROS_INFO("Detected %zu pedestrians in %.3f seconds", pedestrians.size(), processing_time);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // ROI filtering
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

        // Z direction
        pass.setInputCloud(filtered_cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(roi_min_z_, roi_max_z_);
        pass.filter(*filtered_cloud);

        // Voxel grid filtering
        if (voxel_leaf_size_ > 0.0) {
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setInputCloud(filtered_cloud);
            voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
            voxel_filter.filter(*filtered_cloud);
        }

        return filtered_cloud;
    }

    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr>
    removeGround(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr ground(new pcl::PointCloud<pcl::PointXYZ>);

        // RANSAC plane segmentation
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(ground_threshold_);

        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0) {
            ROS_WARN("Could not estimate ground plane");
            obstacles = cloud;
        } else {
            // Extract ground and obstacles
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud);
            extract.setIndices(inliers);
            extract.filter(*ground);

            extract.setNegative(true);
            extract.filter(*obstacles);
        }

        return std::make_pair(obstacles, ground);
    }

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>
    clusterObjects(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;

        if (cloud->empty()) return clusters;

        // Euclidean clustering
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(max_cluster_size_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        // Convert to point cloud clusters
        for (auto& indices : cluster_indices) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (auto& idx : indices.indices) {
                cluster->points.push_back(cloud->points[idx]);
            }
            cluster->width = cluster->points.size();
            cluster->height = 1;
            cluster->is_dense = true;
            clusters.push_back(cluster);
        }

        return clusters;
    }

    std::vector<PedestrianBox> detectPedestrians(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters) {
        std::vector<PedestrianBox> pedestrians;

        for (const auto& cluster : clusters) {
            PedestrianBox pedestrian = analyzePedestrianCandidate(cluster);

            if (isPedestrianValid(pedestrian)) {
                pedestrians.push_back(pedestrian);
            }
        }

        return pedestrians;
    }

    PedestrianBox analyzePedestrianCandidate(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster) {
        PedestrianBox pedestrian;

        if (cluster->empty()) return pedestrian;

        // Get bounding box
        pcl::PointXYZ min_pt, max_pt;
        pcl::getMinMax3D(*cluster, min_pt, max_pt);

        pedestrian.min_point = min_pt;
        pedestrian.max_point = max_pt;
        pedestrian.point_count = cluster->size();

        // Calculate dimensions
        pedestrian.width = max_pt.x - min_pt.x;
        pedestrian.height = max_pt.z - min_pt.z;
        pedestrian.depth = max_pt.y - min_pt.y;

        // Calculate center
        pedestrian.center.x = (max_pt.x + min_pt.x) / 2.0;
        pedestrian.center.y = (max_pt.y + min_pt.y) / 2.0;
        pedestrian.center.z = (max_pt.z + min_pt.z) / 2.0;

        // Calculate confidence based on geometric features
        pedestrian.confidence = calculatePedestrianConfidence(pedestrian);

        return pedestrian;
    }

    bool isPedestrianValid(const PedestrianBox& pedestrian) {
        // Check height constraint
        if (pedestrian.height < min_height_ || pedestrian.height > max_height_) {
            return false;
        }

        // Check width constraint
        double max_horizontal = std::max(pedestrian.width, pedestrian.depth);
        if (max_horizontal < min_width_ || max_horizontal > max_width_) {
            return false;
        }

        // Check depth constraint
        double min_horizontal = std::min(pedestrian.width, pedestrian.depth);
        if (min_horizontal > max_depth_) {
            return false;
        }

        // Check aspect ratio (height should be greater than width)
        if (pedestrian.height < max_horizontal * 1.2) {
            return false;
        }

        // Check point density
        double volume = pedestrian.width * pedestrian.height * pedestrian.depth;
        double point_density = pedestrian.point_count / volume;
        if (point_density < 30.0) {  // Minimum points per cubic meter
            return false;
        }

        return true;
    }

    float calculatePedestrianConfidence(const PedestrianBox& pedestrian) {
        float confidence = 0.0;

        // Height confidence (closer to 1.5m = higher confidence)
        float height_score = 1.0 - std::abs(pedestrian.height - 1.5) / 1.5;
        confidence += 0.4 * std::max(0.0f, height_score);

        // Aspect ratio confidence
        double max_horizontal = std::max(pedestrian.width, pedestrian.depth);
        float aspect_ratio = pedestrian.height / max_horizontal;
        float aspect_score = std::min(1.0f, aspect_ratio / 3.0f);
        confidence += 0.3 * aspect_score;

        // Point density confidence
        double volume = pedestrian.width * pedestrian.height * pedestrian.depth;
        double point_density = pedestrian.point_count / volume;
        float density_score = std::min(1.0f, (float)point_density / 100.0f);
        confidence += 0.3 * density_score;

        return std::min(1.0f, confidence);
    }

    void trackPedestrians(std::vector<PedestrianBox>& current_pedestrians) {
        // Simple tracking based on distance
        const double max_tracking_distance = 2.0;

        for (auto& current : current_pedestrians) {
            double min_distance = std::numeric_limits<double>::max();
            int best_match = -1;

            for (size_t i = 0; i < previous_pedestrians_.size(); ++i) {
                double distance = sqrt(
                    pow(current.center.x - previous_pedestrians_[i].center.x, 2) +
                    pow(current.center.y - previous_pedestrians_[i].center.y, 2) +
                    pow(current.center.z - previous_pedestrians_[i].center.z, 2)
                );

                if (distance < min_distance && distance < max_tracking_distance) {
                    min_distance = distance;
                    best_match = i;
                }
            }

            if (best_match >= 0) {
                current.id = previous_pedestrians_[best_match].id;
            } else {
                current.id = next_id_++;
            }
        }

        previous_pedestrians_ = current_pedestrians;
    }

    void publishResults(const std::vector<PedestrianBox>& pedestrians,
                       const std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr>& segmented_clouds,
                       const std_msgs::Header& header) {

        // Publish bounding boxes
        publishBoundingBoxes(pedestrians, header);

        // Publish visualization markers
        publishVisualizationMarkers(pedestrians, header);

        // Publish point clouds
        publishPointClouds(segmented_clouds, header);
    }

    void publishBoundingBoxes(const std::vector<PedestrianBox>& pedestrians, const std_msgs::Header& header) {
        jsk_recognition_msgs::BoundingBoxArray bbox_array;
        bbox_array.header = header;
        bbox_array.header.frame_id = output_frame_;

        for (const auto& pedestrian : pedestrians) {
            jsk_recognition_msgs::BoundingBox bbox;
            bbox.header = header;
            bbox.header.frame_id = output_frame_;

            bbox.pose.position.x = pedestrian.center.x;
            bbox.pose.position.y = pedestrian.center.y;
            bbox.pose.position.z = pedestrian.center.z;
            bbox.pose.orientation.w = 1.0;

            bbox.dimensions.x = pedestrian.width;
            bbox.dimensions.y = pedestrian.depth;
            bbox.dimensions.z = pedestrian.height;

            bbox.value = pedestrian.confidence;
            bbox.label = pedestrian.id;

            bbox_array.boxes.push_back(bbox);
        }

        bbox_pub_.publish(bbox_array);
    }

    void publishVisualizationMarkers(const std::vector<PedestrianBox>& pedestrians, const std_msgs::Header& header) {
        visualization_msgs::MarkerArray marker_array;

        // Clear previous markers
        visualization_msgs::Marker clear_marker;
        clear_marker.header = header;
        clear_marker.header.frame_id = output_frame_;
        clear_marker.ns = "pedestrian_detection";
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        for (size_t i = 0; i < pedestrians.size(); ++i) {
            const auto& pedestrian = pedestrians[i];

            // Bounding box marker
            visualization_msgs::Marker bbox_marker;
            bbox_marker.header = header;
            bbox_marker.header.frame_id = output_frame_;
            bbox_marker.ns = "pedestrian_bboxes";
            bbox_marker.id = pedestrian.id;
            bbox_marker.type = visualization_msgs::Marker::CUBE;
            bbox_marker.action = visualization_msgs::Marker::ADD;

            bbox_marker.pose.position.x = pedestrian.center.x;
            bbox_marker.pose.position.y = pedestrian.center.y;
            bbox_marker.pose.position.z = pedestrian.center.z;
            bbox_marker.pose.orientation.w = 1.0;

            bbox_marker.scale.x = pedestrian.width;
            bbox_marker.scale.y = pedestrian.depth;
            bbox_marker.scale.z = pedestrian.height;

            // Color based on confidence
            bbox_marker.color.r = 1.0 - pedestrian.confidence;
            bbox_marker.color.g = pedestrian.confidence;
            bbox_marker.color.b = 0.0;
            bbox_marker.color.a = 0.6;

            bbox_marker.lifetime = ros::Duration(0.5);
            marker_array.markers.push_back(bbox_marker);

            // Text marker for ID and confidence
            visualization_msgs::Marker text_marker;
            text_marker.header = header;
            text_marker.header.frame_id = output_frame_;
            text_marker.ns = "pedestrian_labels";
            text_marker.id = pedestrian.id + 1000;
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::ADD;

            text_marker.pose.position.x = pedestrian.center.x;
            text_marker.pose.position.y = pedestrian.center.y;
            text_marker.pose.position.z = pedestrian.center.z + pedestrian.height/2 + 0.3;
            text_marker.pose.orientation.w = 1.0;

            text_marker.scale.z = 0.3;
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;

            text_marker.text = "ID:" + std::to_string(pedestrian.id) +
                              "\nConf:" + std::to_string((int)(pedestrian.confidence * 100)) + "%";
            text_marker.lifetime = ros::Duration(0.5);
            marker_array.markers.push_back(text_marker);
        }

        marker_pub_.publish(marker_array);
    }

    void publishPointClouds(const std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr>& segmented_clouds,
                           const std_msgs::Header& header) {

        // Publish obstacle points
        if (!segmented_clouds.first->empty()) {
            sensor_msgs::PointCloud2 obstacle_msg;
            pcl::toROSMsg(*segmented_clouds.first, obstacle_msg);
            obstacle_msg.header = header;
            obstacle_msg.header.frame_id = output_frame_;
            pedestrian_cloud_pub_.publish(obstacle_msg);
        }

        // Publish ground points
        if (!segmented_clouds.second->empty()) {
            sensor_msgs::PointCloud2 ground_msg;
            pcl::toROSMsg(*segmented_clouds.second, ground_msg);
            ground_msg.header = header;
            ground_msg.header.frame_id = output_frame_;
            ground_cloud_pub_.publish(ground_msg);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "simple_pedestrian_detector");
    SimplePedestrianDetector detector;
    ros::spin();
    return 0;
}