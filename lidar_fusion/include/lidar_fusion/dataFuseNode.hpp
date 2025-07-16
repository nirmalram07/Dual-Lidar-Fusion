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

    //double front_offset_x_, front_offset_y_;
    //double back_offset_x_, back_offset_y_;

    double x_sensor_front, y_sensor_front, x_centre_front, y_centre_front, computed_range_front, theta_c1, theta_front, range_front;
    double x_sensor_back, y_sensor_back, x_centre_back, y_centre_back, computed_range_back, theta_c2, theta_back, range_back;

};
