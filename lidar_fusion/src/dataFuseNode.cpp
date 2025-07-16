#include "lidar_fusion/dataFuseNode.hpp"

using std::placeholders::_1;

DataFuse::DataFuse() : Node("combinedScan"){

    // Front and Back lidar distance from centre
    this->declare_parameter<double>("front_offset_x", 0.25);
    this->declare_parameter<double>("front_offset_y", 0.15);
    this->declare_parameter<double>("back_offset_x", -0.25);
    this->declare_parameter<double>("back_offset_y", -0.15);
    
    lidar_pose_.front_lidar_.offset_x_ = static_cast<float>(this->get_parameter("front_offset_x").as_double());
    lidar_pose_.front_lidar_.offset_y_ = static_cast<float>(this->get_parameter("front_offset_y").as_double());
 
    lidar_pose_.back_lidar_.offset_x_ = static_cast<float>(this->get_parameter("back_offset_x").as_double());
    lidar_pose_.back_lidar_.offset_y_ = static_cast<float>(this->get_parameter("back_offset_y").as_double());

    //RCLCPP_INFO(this->get_logger(), "Back scan data %f", back_offset_y_);
    tf_broadcast_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    //Subsribing to both the lidar's scan data
    scanFront = this->create_subscription<ScanMsg>("scan2", 10,
                std::bind(&DataFuse::frontScanCallback, this, _1));

    scanBack = this->create_subscription<ScanMsg>("scan2", 10,
                std::bind(&DataFuse::backScanCallback, this, _1));

    //Initializing the publisher for combined scan data
    scanCombined = this->create_publisher<ScanMsg>("scan67", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                std::bind(&DataFuse::combinedTfPub, this));
};

void DataFuse::frontScanCallback(const ScanPtr msg){
    //Front Lidar scan data
    scan1_ = msg;
}

void DataFuse::backScanCallback(const ScanPtr msg){
    //Back Lidar scan data
    scan2_ = msg;
}

void DataFuse::combinedTfPub(){

    if (!scan1_ || !scan2_) {
        RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for scan messages...");
        return;
    }

    tf_msg_.header.frame_id = "base_link";
    tf_msg_.header.stamp = this->now();
    tf_msg_.child_frame_id = "combined_scan";
    tf_msg_.transform.translation.x = 0.0;
    tf_msg_.transform.translation.y = 0.0;
    tf_msg_.transform.translation.z = 0.07;
    tf_msg_.transform.rotation.x = 0.0;
    tf_msg_.transform.rotation.y = 0.0;
    tf_msg_.transform.rotation.z = 0.0;
    tf_msg_.transform.rotation.w = 1.0;

    tf_broadcast_->sendTransform(tf_msg_);
    RCLCPP_INFO_ONCE(this->get_logger(),"Publishing combined scan tf");

    combinedScanPub();
}

void DataFuse::combinedScanPub(){

    ScanData = ScanMsg();

    ScanData.header.frame_id = "combined_scan";
    ScanData.header.stamp = this->now();

    ScanData.range_max = 12;
    ScanData.range_min = 0.1;
    ScanData.scan_time = 0.01;
    ScanData.angle_min = 0.0;
    ScanData.angle_max = 2*M_PI;
    ScanData.angle_increment = scan1_->angle_increment;

    int num_beams = static_cast<int>(std::ceil((ScanData.angle_max - ScanData.angle_min) / scan1_->angle_increment));
    ScanData.ranges.resize(num_beams, INFINITY);

    for (size_t i = 0; i < scan1_->ranges.size(); ++i) {

        auto& frontpose_ = lidar_pose_.front_lidar_;
        auto& front_ = laser_geometry_.front_lidar_;

        front_.sensor_angle_ = scan1_->angle_min + i * scan1_->angle_increment;
        front_.sensor_dist_ = scan1_->ranges[i];

        front_.sensor_x_ = front_.sensor_dist_ * cos(front_.sensor_angle_);
        front_.sensor_y_ = front_.sensor_dist_ * sin(front_.sensor_angle_);
        front_.combined_x_ = frontpose_.offset_x_ + front_.sensor_x_;
        front_.combined_y_ = frontpose_.offset_y_ + front_.sensor_y_;
        front_.computed_dist_ = sqrt(front_.combined_x_ * front_.combined_x_ + front_.combined_y_ * front_.combined_y_);
        front_.theta_c1_ = atan2(front_.combined_y_, front_.combined_x_);

        if (front_.theta_c1_ < 0) front_.theta_c1_ += 2 * M_PI;

        int j = static_cast<int>(floor(front_.theta_c1_ / ScanData.angle_increment));

        if (j >= 0 && j < num_beams && front_.computed_dist_ != INFINITY) {
            ScanData.ranges[j] = std::min(ScanData.ranges[j], static_cast<float>(front_.computed_dist_));
        }
    }

    for(size_t i = 0; i<scan2_->ranges.size(); i++){

        auto& backpose_ = lidar_pose_.back_lidar_;
        auto& back_ = laser_geometry_.back_lidar_;

        back_.sensor_angle_ = scan2_->angle_min + i * scan2_->angle_increment;
        back_.sensor_dist_ = scan2_->ranges[i];

        back_.sensor_x_ = back_.sensor_dist_ * cos(back_.sensor_angle_);
        back_.sensor_y_ = back_.sensor_dist_ * sin(back_.sensor_angle_);
        back_.combined_x_ = backpose_.offset_x_ - back_.sensor_x_;
        back_.combined_y_ = backpose_.offset_y_ - back_.sensor_y_;
        back_.computed_dist_ = sqrt(back_.combined_x_ * back_.combined_x_ + back_.combined_y_ * back_.combined_y_);
        back_.theta_c1_ = atan2(back_.combined_y_, back_.combined_x_);

        if(back_.theta_c1_ < 0) back_.theta_c1_ += 2*M_PI;

        int p = static_cast<int>(floor(back_.theta_c1_ / ScanData.angle_increment));

        if (p >= 0 && p < num_beams && back_.computed_dist_ != INFINITY) {
            ScanData.ranges[p] = std::min(ScanData.ranges[p], static_cast<float>(back_.computed_dist_));
        }
    }

    scanCombined->publish(ScanData);
    RCLCPP_INFO_ONCE(this->get_logger(),"Publishing combine scan data");
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto Node = std::make_shared<DataFuse>();
    rclcpp::spin(Node);
    rclcpp::shutdown();
}