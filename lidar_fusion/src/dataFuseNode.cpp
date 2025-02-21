#include "lidar_fusion/dataFuseNode.hpp"

DataFuse::DataFuse() : Node("combinedScan"){

    tf_broadcast_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    scanFront = this->create_subscription<ScanMsg>("scan1", 10,
                std::bind(&DataFuse::frontScanCallback, this, _1));

    scanBack = this->create_subscription<ScanMsg>("scan2", 10,
                std::bind(&DataFuse::backScanCallback, this, _1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                std::bind(&DataFuse::ScanTf, this));
};

void DataFuse::frontScanCallback(const ScanPtr msg){
    
    //Front Lidar scan data
    scanIncrement_1 = msg->angle_increment;
    scanTime_1 = msg->scan_time;
    scanMin_1 = msg->angle_min;
    scanMax_1 = msg->angle_max;
    scanRanges_1 = msg->ranges;
}

void DataFuse::backScanCallback(const ScanPtr msg){
    
    //Back Lidar scan data
    scanIncrement_2 = msg->angle_increment;
    scanTime_2 = msg->scan_time;
    scanMin_2 = msg->angle_min;
    scanMax_2 = msg->angle_max;
    scanRanges_2 = msg->ranges;
}

void DataFuse::ScanTf(){

    tf_msg_.header.frame_id = "chassis1";
    tf_msg_.header.stamp = this->now();
    tf_msg_.child_frame_id = "combined_laser";

    tf_msg_.transform.translation.x = 0.0;
    tf_msg_.transform.translation.y = 0.0;
    tf_msg_.transform.translation.z = -0.05;

    tf_msg_.transform.rotation.x = 0.0;
    tf_msg_.transform.rotation.y = 0.0;
    tf_msg_.transform.rotation.z = 0.0;
    tf_msg_.transform.rotation.w = 0.0;

    tf_broadcast_->sendTransform(tf_msg_);
    
    RCLCPP_INFO_ONCE(this->get_logger(),"Published dummy tf");
}