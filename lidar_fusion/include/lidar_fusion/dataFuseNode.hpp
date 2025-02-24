#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2/LinearMath/Transform.h>

#include <vector>
#include <cmath>

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
    bool is_value_overlapping(double theta);

    geometry_msgs::msg::TransformStamped tf_msg_;
    ScanMsg::SharedPtr scan1, scan2;

    float scanIncrement_1, scanMin_1, scanMax_1, scanTime_1;
    float scanIncrement_2, scanMin_2, scanMax_2, scanTime_2;

    std::vector<float> scanRanges_1, scanRanges_2;

    rclcpp::Subscription<ScanMsg>::SharedPtr scanFront, scanBack;
    rclcpp::Publisher<ScanMsg>::SharedPtr scanCombined;

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcast_;

    double front_offset_x_, front_offset_y_, front_min_angle_, front_max_angle_;
    double back_offset_x_, back_offset_y_, back_min_angle_, back_max_angle_;

};
