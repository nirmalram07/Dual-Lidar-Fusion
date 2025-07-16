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

using ScanMsg = sensor_msgs::msg::LaserScan;
using ScanPtr = ScanMsg::SharedPtr;

struct LidarOffsets{
    float offset_x_;
    float offset_y_;
};

struct LidarPositions{
    LidarOffsets front_lidar_;
    LidarOffsets back_lidar_;
};

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

struct LidarInfo{
    LidarMetrics front_lidar_;
    LidarMetrics back_lidar_;
};

class DataFuse : public rclcpp::Node{

public:
    DataFuse();

private:

    void frontScanCallback(const ScanPtr msg);
    void backScanCallback(const ScanPtr msg);
    void combinedTfPub();
    void combinedScanPub();

    geometry_msgs::msg::TransformStamped tf_msg_;
    ScanMsg::SharedPtr scan1_, scan2_;
    ScanMsg ScanData;

    rclcpp::Subscription<ScanMsg>::SharedPtr scanFront, scanBack;
    rclcpp::Publisher<ScanMsg>::SharedPtr scanCombined;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcast_;
    rclcpp::TimerBase::SharedPtr timer_;

    LidarPositions lidar_pose_;
    LidarInfo laser_geometry_;
};
