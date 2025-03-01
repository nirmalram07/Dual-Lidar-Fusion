#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Transform.h>

#include <vector>
#include <cmath>
#include <algorithm>

using ScanMsg = sensor_msgs::msg::LaserScan;
using ScanPtr = ScanMsg::SharedPtr;
constexpr auto _1 = std::placeholders::_1;

class DataFuse : public rclcpp::Node{
public:
    DataFuse();

private:

    void frontScanCallback(const ScanPtr msg);
    void backScanCallback(const ScanPtr msg);
    void CombinedScan();

    geometry_msgs::msg::TransformStamped tf_msg_;
    ScanMsg::SharedPtr scan1, scan2;
    ScanMsg ScanData;

    rclcpp::Subscription<ScanMsg>::SharedPtr scanFront, scanBack;
    rclcpp::Publisher<ScanMsg>::SharedPtr scanCombined;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcast_;
    rclcpp::TimerBase::SharedPtr timer_;

    double front_offset_x_, front_offset_y_;
    double back_offset_x_, back_offset_y_;

    double x_sensor_front, y_sensor_front, x_centre_front, y_centre_front, computed_range_front, theta_c1, theta_front, range_front;
    double x_sensor_back, y_sensor_back, x_centre_back, y_centre_back, computed_range_back, theta_c2, theta_back, range_back;

};
