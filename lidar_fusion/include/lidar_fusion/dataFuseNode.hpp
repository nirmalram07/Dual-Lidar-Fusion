#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Transform.h>

/**
 * @brief Alias for LaserScan msg.
 */
using ScanMsg = sensor_msgs::msg::LaserScan;

/**
 * @brief Alias for a shared pointer to LaserScan.
 */
using ScanPtr = ScanMsg::SharedPtr;

/**
 * @brief Structure to hold offset positions for a LIDAR sensor.
 */
struct LidarOffsets{
    float offset_x_;
    float offset_y_;
};

/**
 * @brief Structure to store offset positions for both front and back LIDAR sensors.
 */
struct LidarPositions{
    LidarOffsets front_lidar_;
    LidarOffsets back_lidar_;
};

/**
 * @brief Structure to hold geometric and computed metrics for a LIDAR sensor.
 */
struct LidarMetrics{
    float sensor_angle_;
    float sensor_dist_;
    float sensor_x_;
    float sensor_y_;
    float combined_x_;
    float combined_y_;
    float computed_dist_;
    float theta_c1_;
};

/**
 * @brief Structure to store metrics for both front and back LIDAR sensors.
 */
struct LidarInfo{
    LidarMetrics front_lidar_;
    LidarMetrics back_lidar_;
};

/**
 * @class Main class which handles the data subscription and publishing of combined data
 * 
 * @brief ROS 2 node to fuse scan data from front and back LIDAR sensors and publish a combined scan.
 * 
 * @details This class subscribes to scan data from two LIDAR sensors, transforms their data into a common frame,
 * and publishes a combined LaserScan message. It also broadcasts a transform for the combined scan frame.
 */
class DataFuse : public rclcpp::Node{

public:
    /**
     * @brief Constructor for the DataFuse node.
     */
    DataFuse();

private:

    void frontScanCallback(const ScanPtr msg);
    void backScanCallback(const ScanPtr msg);

    /**
     * @brief Publishes the transform for the combined scan frame.
     */
    void combinedTfPub();

    /**
     * @brief Publishes the combined scan data from front and back LIDARs.
     */
    void combinedScanPub();

    // Transform message for broadcasting the combined scan frame.
    geometry_msgs::msg::TransformStamped tf_msg_;

    //Shared pointers to store the latest front and back LIDAR scan messages.
    ScanMsg::SharedPtr scan1_, scan2_;

    //Combined LaserScan message to be published.
    ScanMsg ScanData;

    //Subscription to the front & back LIDAR scan topic.
    rclcpp::Subscription<ScanMsg>::SharedPtr scanFront, scanBack;

    //Subscription to the back LIDAR scan topic.
    rclcpp::Publisher<ScanMsg>::SharedPtr scanCombined;

    //Publisher for the combined scan topic.
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcast_;

    //TF2 broadcaster for the combined scan frame.
    rclcpp::TimerBase::SharedPtr timer_;

    //Offset positions for front and back LIDARs.
    LidarPositions lidar_pose_;

    //Geometric metrics for front and back LIDAR scans.
    LidarInfo laser_geometry_;
};
