#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <vector>

using ScanMsg = sensor_msgs::msg::LaserScan;
using ScanPtr = ScanMsg::SharedPtr;
constexpr auto _1 = std::placeholders::_1;

class DataFuse : public rclcpp::Node{
public:
    DataFuse();

private:

    void frontScanCallback(const ScanPtr msg);
    void backScanCallback(const ScanPtr msg);
    void ScanTf();

    geometry_msgs::msg::TransformStamped tf_msg_;
    ScanMsg scan1, scan2;

    float scanIncrement_1, scanMin_1, scanMax_1, scanTime_1;
    float scanIncrement_2, scanMin_2, scanMax_2, scanTime_2;

    std::vector<float> scanRanges_1, scanRanges_2;

    rclcpp::Subscription<ScanMsg>::SharedPtr scanFront, scanBack;

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcast_;
};