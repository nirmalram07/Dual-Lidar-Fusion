#include "lidar_fusion/dataFuseNode.hpp"

DataFuse::DataFuse() : Node("combinedScan"){

    // Front LiDAR parameters (front-left)

    this->declare_parameter<double>("front_offset_x", 0.25);
    this->declare_parameter<double>("front_offset_y", 0.15);
    this->declare_parameter<double>("back_offset_x", -0.25);
    this->declare_parameter<double>("back_offset_y", -0.15);
    
    front_offset_x_ = this->get_parameter("front_offset_x").as_double();
    front_offset_y_ = this->get_parameter("front_offset_y").as_double();
 
    back_offset_x_ = this->get_parameter("back_offset_x").as_double();
    back_offset_y_ = this->get_parameter("back_offset_y").as_double();

    RCLCPP_INFO(this->get_logger(), "Back scan data %f", back_offset_y_);

    tf_broadcast_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    scanFront = this->create_subscription<ScanMsg>("scan1", 10,
                std::bind(&DataFuse::frontScanCallback, this, _1));

    scanBack = this->create_subscription<ScanMsg>("scan2", 10,
                std::bind(&DataFuse::backScanCallback, this, _1));

    scanCombined = this->create_publisher<ScanMsg>("scan67", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(5),
                std::bind(&DataFuse::CombinedScan, this));
};

void DataFuse::frontScanCallback(const ScanPtr msg){
    
    //Front Lidar scan data
    scan1 = msg;
}

void DataFuse::backScanCallback(const ScanPtr msg){
    
    //Back Lidar scan data
    scan2 = msg;
}

void DataFuse::CombinedScan(){

    if (!scan1 || !scan2) {
        RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for scan messages...");
        return;
    }

    tf_msg_.header.frame_id = "base_link";
    tf_msg_.header.stamp = this->now();
    tf_msg_.child_frame_id = "combined_laser";

    tf_msg_.transform.translation.x = 0.0;
    tf_msg_.transform.translation.y = 0.0;
    tf_msg_.transform.translation.z = 0.07;

    tf_msg_.transform.rotation.x = 0.0;
    tf_msg_.transform.rotation.y = 0.0;
    tf_msg_.transform.rotation.z = 0.0;
    tf_msg_.transform.rotation.w = 1.0;

    tf_broadcast_->sendTransform(tf_msg_);

    RCLCPP_INFO_ONCE(this->get_logger(),"Published dummy tf");

    auto ScanData = ScanMsg();

    ScanData.header.frame_id = "combined_laser";
    ScanData.header.stamp = this->now();

    ScanData.range_max = 12;
    ScanData.range_min = 0.1;
    ScanData.scan_time = 0.01;
    ScanData.angle_min = 0.0;
    ScanData.angle_increment = scan1->angle_increment;
    ScanData.angle_max = 2*M_PI;

    int num_beams1 = static_cast<int>(std::ceil((ScanData.angle_max - ScanData.angle_min) / scan1->angle_increment));
    ScanData.ranges.resize(num_beams1, INFINITY);

    for (size_t i = 0; i < scan1->ranges.size(); ++i) {

    double theta = scan1->angle_min + i * scan1->angle_increment;
    double range_value = scan1->ranges[i];

    double x_sensor1 = range_value * cos(theta);
    double y_sensor1 = range_value * sin(theta);
    double x_centre1 = front_offset_x_ + x_sensor1;
    double y_centre1 = front_offset_y_ + y_sensor1;

    double computed_range1 = sqrt(x_centre1 * x_centre1 + y_centre1 * y_centre1);
    double theta_c = atan2(y_centre1, x_centre1);

    if (theta_c < 0) theta_c += 2 * M_PI;

    int j = static_cast<int>(floor(theta_c / ScanData.angle_increment));

    if (j >= 0 && j < num_beams1 && computed_range1 != INFINITY) {
        ScanData.ranges[j] = std::min(ScanData.ranges[j], static_cast<float>(computed_range1));
    }

    for(size_t i = 0; i<scan2->ranges.size(); i++){

        double theta_back = scan2->angle_min + i * scan2->angle_increment;
        double range_back = scan2->ranges[i];

        double x_sensor2 = range_back * cos(theta_back);
        double y_sensor2 = range_back * sin(theta_back);

        double x_centre2 = back_offset_x_ - x_sensor2;
        double y_centre2 = back_offset_y_ - y_sensor2;

        double computed_range2 = sqrt(x_centre2*x_centre2 + y_centre2*y_centre2);
        double theta_c2 = atan2(y_centre2, x_centre2);

        if(theta_c2 < 0) theta_c2 += 2*M_PI;

        int p = static_cast<int>(floor(theta_c2 / ScanData.angle_increment));

        if (p >= 0 && p < num_beams1 && computed_range2 != INFINITY) {
        ScanData.ranges[p] = std::min(ScanData.ranges[p], static_cast<float>(computed_range2));
        }
    }
}
    scanCombined->publish(ScanData);
    RCLCPP_INFO_ONCE(this->get_logger(),"Publishing Combined Scan data");
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto Node = std::make_shared<DataFuse>();
    rclcpp::spin(Node);
    rclcpp::shutdown();
}