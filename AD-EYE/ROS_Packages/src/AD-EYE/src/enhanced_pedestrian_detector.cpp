/*
 * enhanced_pedestrian_detector.cpp
 * Enhanced LiDAR-based pedestrian detection node
 * Combines clustering, PCA-based analysis, and sophisticated tracking
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
#include <pcl/common/pca.h>

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <algorithm>
#include <unordered_map>
#include <deque>

struct TrackedPedestrian {
    int id;
    Eigen::Vector3f position;
    Eigen::Vector3f velocity;
    Eigen::Matrix3f orientation;
    Eigen::Vector3f dimensions;
    float confidence;
    int point_count;
    int age;
    int tracking_age;

    TrackedPedestrian() : id(0), confidence(0.0f), point_count(0), age(1), tracking_age(0) {
        position.setZero();
        velocity.setZero();
        orientation.setIdentity();
        dimensions.setZero();
    }
};

class EnhancedPedestrianDetector {
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
    double min_aspect_ratio_;
    double max_aspect_ratio_;
    double min_point_density_;

    // ROI parameters
    double roi_min_x_, roi_max_x_;
    double roi_min_y_, roi_max_y_;
    double roi_min_z_, roi_max_z_;

    // Tracking parameters
    int tracking_history_size_;
    double max_tracking_distance_;
    double velocity_smoothing_factor_;
    double position_smoothing_factor_;
    int max_tracking_age_;

    // Tracking data
    std::unordered_map<int, TrackedPedestrian> tracked_pedestrians_;
    std::deque<std::vector<std::pair<int, Eigen::Vector3f>>> position_history_;
    int next_id_;
    ros::Time last_detection_time_;

public:
    EnhancedPedestrianDetector() : private_nh_("~"), next_id_(1) {
        initializeParameters();
        initializePublishersSubscribers();
        last_detection_time_ = ros::Time::now();

        ROS_INFO("Enhanced Pedestrian Detector initialized");
        ROS_INFO("Listening to: %s", input_topic_.c_str());
        ROS_INFO("Detection constraints - Height:[%.2f-%.2f] Width:[%.2f-%.2f] Depth:[0-%.2f]",
                 min_height_, max_height_, min_width_, max_width_, max_depth_);
        ROS_INFO("Tracking - Max distance: %.2f, History size: %d", max_tracking_distance_, tracking_history_size_);
    }

    void initializeParameters() {
        // Topic parameters
        private_nh_.param<std::string>("input_topic", input_topic_, "/filtered_points");
        private_nh_.param<std::string>("output_frame", output_frame_, "base_link");
        private_nh_.param<bool>("enable_ground_removal", enable_ground_removal_, true);

        // Processing parameters
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
        private_nh_.param<double>("min_aspect_ratio", min_aspect_ratio_, 1.5);
        private_nh_.param<double>("max_aspect_ratio", max_aspect_ratio_, 4.0);
        private_nh_.param<double>("min_point_density", min_point_density_, 50.0);

        // ROI parameters
        private_nh_.param<double>("roi_min_x", roi_min_x_, -2.0);
        private_nh_.param<double>("roi_max_x", roi_max_x_, 30.0);
        private_nh_.param<double>("roi_min_y", roi_min_y_, -15.0);
        private_nh_.param<double>("roi_max_y", roi_max_y_, 15.0);
        private_nh_.param<double>("roi_min_z", roi_min_z_, -0.5);
        private_nh_.param<double>("roi_max_z", roi_max_z_, 2.5);

        // Tracking parameters
        private_nh_.param<int>("tracking_history_size", tracking_history_size_, 5);
        private_nh_.param<double>("max_tracking_distance", max_tracking_distance_, 2.0);
        private_nh_.param<double>("velocity_smoothing_factor", velocity_smoothing_factor_, 0.3);
        private_nh_.param<double>("position_smoothing_factor", position_smoothing_factor_, 0.3);
        private_nh_.param<int>("max_tracking_age", max_tracking_age_, 10);
    }

    void initializePublishersSubscribers() {
        cloud_sub_ = nh_.subscribe(input_topic_, 1, &EnhancedPedestrianDetector::cloudCallback, this);

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

        // Detect pedestrians using PCA analysis
        std::vector<TrackedPedestrian> detected_pedestrians = detectPedestrians(clusters);

        // Update tracking
        updateTracking(detected_pedestrians, input_cloud->header.stamp);

        // Publish results
        publishResults(segmented_clouds, input_cloud->header);

        auto processing_time = (ros::Time::now() - start_time).toSec();
        ROS_INFO("Detected %zu pedestrians, tracking %zu objects in %.3f seconds",
                 detected_pedestrians.size(), tracked_pedestrians_.size(), processing_time);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

        // ROI filtering only (voxel filtering already done by upstream filter)
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

    void computePCA(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                   Eigen::Vector3f& dimensions,
                   Eigen::Matrix3f& orientation,
                   Eigen::Vector3f& center) {

        if (cloud->empty()) {
            dimensions.setZero();
            orientation.setIdentity();
            center.setZero();
            return;
        }

        // Compute centroid
        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, centroid);
        center = centroid.head<3>();

        try {
            // Compute PCA
            pcl::PCA<pcl::PointXYZ> pca;
            pca.setInputCloud(cloud);

            // Get the principal components
            orientation = pca.getEigenVectors();

            // Project points onto principal components to get dimensions
            Eigen::Vector3f min_point(FLT_MAX, FLT_MAX, FLT_MAX);
            Eigen::Vector3f max_point(-FLT_MAX, -FLT_MAX, -FLT_MAX);

            for (const auto& point : cloud->points) {
                Eigen::Vector3f p(point.x, point.y, point.z);
                Eigen::Vector3f projected = orientation.transpose() * (p - center);
                min_point = min_point.cwiseMin(projected);
                max_point = max_point.cwiseMax(projected);
            }

            dimensions = max_point - min_point;
        } catch (const std::exception& e) {
            ROS_WARN("PCA computation failed: %s. Using bounding box fallback.", e.what());

            // Fallback to simple bounding box
            pcl::PointXYZ min_pt, max_pt;
            pcl::getMinMax3D(*cloud, min_pt, max_pt);

            dimensions[0] = max_pt.x - min_pt.x;
            dimensions[1] = max_pt.y - min_pt.y;
            dimensions[2] = max_pt.z - min_pt.z;
            orientation.setIdentity();
        }
    }

    std::vector<TrackedPedestrian> detectPedestrians(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters) {
        std::vector<TrackedPedestrian> pedestrians;

        for (const auto& cluster : clusters) {
            TrackedPedestrian pedestrian = analyzePedestrianCandidate(cluster);

            if (isPedestrianValid(pedestrian)) {
                pedestrians.push_back(pedestrian);
            }
        }

        return pedestrians;
    }

    TrackedPedestrian analyzePedestrianCandidate(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster) {
        TrackedPedestrian pedestrian;

        if (cluster->empty()) return pedestrian;

        pedestrian.point_count = cluster->size();

        // Compute PCA-based bounding box
        computePCA(cluster, pedestrian.dimensions, pedestrian.orientation, pedestrian.position);

        // Calculate confidence based on geometric features and point density
        pedestrian.confidence = calculatePedestrianConfidence(pedestrian);

        return pedestrian;
    }

    bool isPedestrianValid(const TrackedPedestrian& pedestrian) {
        // Check height constraint
        if (pedestrian.dimensions(2) < min_height_ || pedestrian.dimensions(2) > max_height_) {
            return false;
        }

        // Check width constraints - use the two horizontal dimensions
        double width = pedestrian.dimensions(0);
        double depth = pedestrian.dimensions(1);
        double max_horizontal = std::max(width, depth);
        double min_horizontal = std::min(width, depth);

        if (max_horizontal < min_width_ || max_horizontal > max_width_) {
            return false;
        }

        if (min_horizontal > max_depth_) {
            return false;
        }

        // Check aspect ratio (height should be greater than horizontal dimensions)
        double aspect_ratio = pedestrian.dimensions(2) / max_horizontal;
        if (aspect_ratio < min_aspect_ratio_ || aspect_ratio > max_aspect_ratio_) {
            return false;
        }

        // Check point density
        double volume = pedestrian.dimensions(0) * pedestrian.dimensions(1) * pedestrian.dimensions(2);
        if (volume <= 0) return false;

        double point_density = pedestrian.point_count / volume;
        if (point_density < min_point_density_) {
            return false;
        }

        return true;
    }

    float calculatePedestrianConfidence(const TrackedPedestrian& pedestrian) {
        float confidence = 0.0;

        // Height confidence (closer to typical human height = higher confidence)
        float ideal_height = 1.7f;
        float height_score = 1.0f - std::abs(pedestrian.dimensions(2) - ideal_height) / ideal_height;
        confidence += 0.3f * std::max(0.0f, height_score);

        // Aspect ratio confidence
        double max_horizontal = std::max(pedestrian.dimensions(0), pedestrian.dimensions(1));
        float aspect_ratio = pedestrian.dimensions(2) / max_horizontal;
        float ideal_aspect = 2.5f;
        float aspect_score = 1.0f - std::abs(aspect_ratio - ideal_aspect) / ideal_aspect;
        confidence += 0.3f * std::max(0.0f, aspect_score);

        // Point density confidence
        double volume = pedestrian.dimensions(0) * pedestrian.dimensions(1) * pedestrian.dimensions(2);
        double point_density = pedestrian.point_count / volume;
        float density_score = std::min(1.0f, (float)point_density / 100.0f);
        confidence += 0.2f * density_score;

        // Symmetry confidence - pedestrians should be roughly symmetric
        double width_depth_ratio = std::max(pedestrian.dimensions(0), pedestrian.dimensions(1)) /
                                  std::min(pedestrian.dimensions(0), pedestrian.dimensions(1));
        float symmetry_score = 1.0f / width_depth_ratio;  // Closer to 1 = more symmetric
        confidence += 0.2f * std::min(1.0f, symmetry_score);

        return std::min(1.0f, confidence);
    }

    double computeIOU(const TrackedPedestrian& obj1, const TrackedPedestrian& obj2) {
        double distance = (obj1.position - obj2.position).norm();
        double max_dimension = std::max({obj1.dimensions.maxCoeff(), obj2.dimensions.maxCoeff()});
        return std::exp(-distance * distance / (2.0 * max_dimension * max_dimension));
    }

    void updateTracking(const std::vector<TrackedPedestrian>& detected_pedestrians, const ros::Time& current_time) {
        double dt = (current_time - last_detection_time_).toSec();
        dt = std::max(0.01, std::min(dt, 0.5));  // Clamp dt to reasonable values

        // Update position history
        std::vector<std::pair<int, Eigen::Vector3f>> current_positions;
        for (const auto& ped : detected_pedestrians) {
            current_positions.push_back({-1, ped.position});  // -1 indicates unassigned
        }
        position_history_.push_back(current_positions);
        if (position_history_.size() > tracking_history_size_) {
            position_history_.pop_front();
        }

        // Create cost matrix for assignment
        std::vector<std::vector<double>> cost_matrix;
        std::vector<int> tracked_ids;

        for (const auto& tracked_pair : tracked_pedestrians_) {
            tracked_ids.push_back(tracked_pair.first);
        }

        cost_matrix.resize(tracked_ids.size());
        for (size_t i = 0; i < tracked_ids.size(); ++i) {
            cost_matrix[i].resize(detected_pedestrians.size());
            for (size_t j = 0; j < detected_pedestrians.size(); ++j) {
                double distance = (tracked_pedestrians_[tracked_ids[i]].position - detected_pedestrians[j].position).norm();
                cost_matrix[i][j] = distance;
            }
        }

        // Simple greedy assignment (could be replaced with Hungarian algorithm)
        std::vector<bool> detection_assigned(detected_pedestrians.size(), false);
        std::vector<bool> track_assigned(tracked_ids.size(), false);

        for (size_t i = 0; i < tracked_ids.size(); ++i) {
            double min_distance = max_tracking_distance_;
            int best_detection = -1;

            for (size_t j = 0; j < detected_pedestrians.size(); ++j) {
                if (!detection_assigned[j] && cost_matrix[i][j] < min_distance) {
                    min_distance = cost_matrix[i][j];
                    best_detection = j;
                }
            }

            if (best_detection >= 0) {
                // Update existing track
                int track_id = tracked_ids[i];
                auto& tracked_ped = tracked_pedestrians_[track_id];
                const auto& detected_ped = detected_pedestrians[best_detection];

                // Update velocity with smoothing
                Eigen::Vector3f new_velocity = (detected_ped.position - tracked_ped.position) / dt;
                tracked_ped.velocity = (1.0f - velocity_smoothing_factor_) * tracked_ped.velocity +
                                      velocity_smoothing_factor_ * new_velocity;

                // Update position with smoothing
                tracked_ped.position = (1.0f - position_smoothing_factor_) * tracked_ped.position +
                                      position_smoothing_factor_ * detected_ped.position;

                // Update other properties
                tracked_ped.orientation = detected_ped.orientation;
                tracked_ped.dimensions = detected_ped.dimensions;
                tracked_ped.confidence = std::min(1.0f, tracked_ped.confidence + 0.1f);
                tracked_ped.point_count = detected_ped.point_count;
                tracked_ped.tracking_age++;

                detection_assigned[best_detection] = true;
                track_assigned[i] = true;
            }
        }

        // Age unmatched tracks
        for (size_t i = 0; i < tracked_ids.size(); ++i) {
            if (!track_assigned[i]) {
                auto& tracked_ped = tracked_pedestrians_[tracked_ids[i]];
                tracked_ped.age++;
                tracked_ped.confidence = std::max(0.0f, tracked_ped.confidence - 0.1f);

                // Predict position based on velocity
                tracked_ped.position += tracked_ped.velocity * dt;
            }
        }

        // Create new tracks for unassigned detections
        for (size_t j = 0; j < detected_pedestrians.size(); ++j) {
            if (!detection_assigned[j]) {
                TrackedPedestrian new_track = detected_pedestrians[j];
                new_track.id = next_id_++;
                new_track.velocity.setZero();
                new_track.age = 1;
                new_track.tracking_age = 1;
                tracked_pedestrians_[new_track.id] = new_track;
            }
        }

        // Remove old tracks
        auto it = tracked_pedestrians_.begin();
        while (it != tracked_pedestrians_.end()) {
            if (it->second.age > max_tracking_age_ || it->second.confidence < 0.1f) {
                it = tracked_pedestrians_.erase(it);
            } else {
                ++it;
            }
        }

        last_detection_time_ = current_time;
    }

    void publishResults(const std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr>& segmented_clouds,
                       const std_msgs::Header& header) {

        // Convert tracked pedestrians to vector for publishing
        std::vector<TrackedPedestrian> current_pedestrians;
        for (const auto& pair : tracked_pedestrians_) {
            if (pair.second.confidence > 0.3f) {  // Only publish confident detections
                current_pedestrians.push_back(pair.second);
            }
        }

        // Publish bounding boxes
        publishBoundingBoxes(current_pedestrians, header);

        // Publish visualization markers
        publishVisualizationMarkers(current_pedestrians, header);

        // Publish point clouds
        publishPointClouds(segmented_clouds, header);
    }

    void publishBoundingBoxes(const std::vector<TrackedPedestrian>& pedestrians, const std_msgs::Header& header) {
        jsk_recognition_msgs::BoundingBoxArray bbox_array;
        bbox_array.header = header;
        bbox_array.header.frame_id = output_frame_;

        for (const auto& pedestrian : pedestrians) {
            jsk_recognition_msgs::BoundingBox bbox;
            bbox.header = header;
            bbox.header.frame_id = output_frame_;

            bbox.pose.position.x = pedestrian.position.x();
            bbox.pose.position.y = pedestrian.position.y();
            bbox.pose.position.z = pedestrian.position.z();

            // Convert orientation matrix to quaternion
            Eigen::Quaternionf q(pedestrian.orientation);
            bbox.pose.orientation.x = q.x();
            bbox.pose.orientation.y = q.y();
            bbox.pose.orientation.z = q.z();
            bbox.pose.orientation.w = q.w();

            bbox.dimensions.x = pedestrian.dimensions.x();
            bbox.dimensions.y = pedestrian.dimensions.y();
            bbox.dimensions.z = pedestrian.dimensions.z();

            bbox.value = pedestrian.confidence;
            bbox.label = pedestrian.id;

            bbox_array.boxes.push_back(bbox);
        }

        bbox_pub_.publish(bbox_array);
    }

    void publishVisualizationMarkers(const std::vector<TrackedPedestrian>& pedestrians, const std_msgs::Header& header) {
        visualization_msgs::MarkerArray marker_array;

        // Clear previous markers
        visualization_msgs::Marker clear_marker;
        clear_marker.header = header;
        clear_marker.header.frame_id = output_frame_;
        clear_marker.ns = "pedestrian_detection";
        clear_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        for (const auto& pedestrian : pedestrians) {
            // Bounding box marker
            visualization_msgs::Marker bbox_marker;
            bbox_marker.header = header;
            bbox_marker.header.frame_id = output_frame_;
            bbox_marker.ns = "pedestrian_bboxes";
            bbox_marker.id = pedestrian.id;
            bbox_marker.type = visualization_msgs::Marker::CUBE;
            bbox_marker.action = visualization_msgs::Marker::ADD;

            bbox_marker.pose.position.x = pedestrian.position.x();
            bbox_marker.pose.position.y = pedestrian.position.y();
            bbox_marker.pose.position.z = pedestrian.position.z();

            // Convert orientation matrix to quaternion
            Eigen::Quaternionf q(pedestrian.orientation);
            bbox_marker.pose.orientation.x = q.x();
            bbox_marker.pose.orientation.y = q.y();
            bbox_marker.pose.orientation.z = q.z();
            bbox_marker.pose.orientation.w = q.w();

            bbox_marker.scale.x = pedestrian.dimensions.x();
            bbox_marker.scale.y = pedestrian.dimensions.y();
            bbox_marker.scale.z = pedestrian.dimensions.z();

            // Color based on confidence
            bbox_marker.color.r = 1.0 - pedestrian.confidence;
            bbox_marker.color.g = pedestrian.confidence;
            bbox_marker.color.b = 0.0;
            bbox_marker.color.a = 0.6;

            bbox_marker.lifetime = ros::Duration(0.5);
            marker_array.markers.push_back(bbox_marker);

            // Velocity arrow marker
            if (pedestrian.velocity.norm() > 0.1) {
                visualization_msgs::Marker arrow_marker;
                arrow_marker.header = header;
                arrow_marker.header.frame_id = output_frame_;
                arrow_marker.ns = "pedestrian_velocities";
                arrow_marker.id = pedestrian.id + 2000;
                arrow_marker.type = visualization_msgs::Marker::ARROW;
                arrow_marker.action = visualization_msgs::Marker::ADD;

                geometry_msgs::Point start, end;
                start.x = pedestrian.position.x();
                start.y = pedestrian.position.y();
                start.z = pedestrian.position.z() + pedestrian.dimensions.z() / 2;

                Eigen::Vector3f velocity_scaled = pedestrian.velocity * 0.5f;  // Scale for visualization
                end.x = start.x + velocity_scaled.x();
                end.y = start.y + velocity_scaled.y();
                end.z = start.z + velocity_scaled.z();

                arrow_marker.points.push_back(start);
                arrow_marker.points.push_back(end);

                arrow_marker.scale.x = 0.1;  // Arrow shaft diameter
                arrow_marker.scale.y = 0.2;  // Arrow head diameter

                arrow_marker.color.r = 0.0;
                arrow_marker.color.g = 0.0;
                arrow_marker.color.b = 1.0;
                arrow_marker.color.a = 0.8;

                arrow_marker.lifetime = ros::Duration(0.5);
                marker_array.markers.push_back(arrow_marker);
            }

            // Text marker for ID, confidence, and tracking info
            visualization_msgs::Marker text_marker;
            text_marker.header = header;
            text_marker.header.frame_id = output_frame_;
            text_marker.ns = "pedestrian_labels";
            text_marker.id = pedestrian.id + 1000;
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::Marker::ADD;

            text_marker.pose.position.x = pedestrian.position.x();
            text_marker.pose.position.y = pedestrian.position.y();
            text_marker.pose.position.z = pedestrian.position.z() + pedestrian.dimensions.z()/2 + 0.3;
            text_marker.pose.orientation.w = 1.0;

            text_marker.scale.z = 0.3;
            text_marker.color.r = 1.0;
            text_marker.color.g = 1.0;
            text_marker.color.b = 1.0;
            text_marker.color.a = 1.0;

            text_marker.text = "ID:" + std::to_string(pedestrian.id) +
                              "\nConf:" + std::to_string((int)(pedestrian.confidence * 100)) + "%" +
                              "\nAge:" + std::to_string(pedestrian.tracking_age) +
                              "\nVel:" + std::to_string((int)(pedestrian.velocity.norm() * 10)) + "/10";
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
    ros::init(argc, argv, "enhanced_pedestrian_detector");
    EnhancedPedestrianDetector detector;
    ros::spin();
    return 0;
}