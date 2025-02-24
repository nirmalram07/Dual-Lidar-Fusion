#include "lidar_fusion/dataFuseNode.hpp"

DataFuse::DataFuse() : Node("combinedScan"){

    // Front LiDAR parameters (front-left)
    front_offset_x_ = 0.25;
    front_offset_y_ = 0.15;
    front_min_angle_ = 4.64;
    front_max_angle_ = 8.74;

    // Back LiDAR parameters (back-right)
    back_offset_x_ = -0.25;
    back_offset_y_ = -0.15;
    back_min_angle_ = 1.51;
    back_max_angle_ = 5.59;

    tf_broadcast_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    scanFront = this->create_subscription<ScanMsg>("scan1", 10,
                std::bind(&DataFuse::frontScanCallback, this, _1));

    scanBack = this->create_subscription<ScanMsg>("scan2", 10,
                std::bind(&DataFuse::backScanCallback, this, _1));

    scanCombined = this->create_publisher<ScanMsg>("scan67", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                std::bind(&DataFuse::ScanTf, this));
};

void DataFuse::frontScanCallback(const ScanPtr msg){
    
    //Front Lidar scan data
    scan1 = msg;
}

void DataFuse::backScanCallback(const ScanPtr msg){
    
    //Back Lidar scan data
    scan2 = msg;
}

void DataFuse::ScanTf(){

    if (!scan1 || !scan2) {
        RCLCPP_WARN(this->get_logger(), "Waiting for scan messages...");
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
    ScanData.scan_time = 0.05;
    ScanData.angle_min = 0.0;
    ScanData.angle_max = 2*M_PI;
    ScanData.angle_increment = 2*M_PI/360;
    ScanData.ranges.resize(360, INFINITY);

    for(size_t i = 0; i < 360; ++i) {
        const double theta = ScanData.angle_min + i * ScanData.angle_increment;

        if(is_value_overlapping(theta)){
            int front_scan = static_cast<int>((theta-scan1->angle_min)/scan1->angle_increment);
            front_scan %= 360;

            int back_scan = static_cast<int>((theta-scan2->angle_min)/scan2->angle_increment);
            back_scan %= 360;

            float front_angle = scan1->ranges[front_scan];
            float back_angle = scan2->ranges[back_scan];

            tf2::Vector3 front_point(
                front_angle * cos(theta) + front_offset_x_,
                front_angle * sin(theta) + front_offset_y_,
                0.0
            );
            float front_dist = front_point.length();

            // Transform back lidar measurement to center frame
            tf2::Vector3 back_point(
                back_angle * cos(theta) + back_offset_x_,
                back_angle * sin(theta) + back_offset_y_,
                0.0
            );
            float back_dist = back_point.length();

            // Select valid minimum distance
            bool front_valid = (front_angle >= ScanData.range_min) && (front_angle <= ScanData.range_max);
            bool back_valid = (back_angle >= ScanData.range_min) && (back_angle <= ScanData.range_max);

            if(front_valid && back_valid) {
                ScanData.ranges[i] = std::min(front_dist, back_dist);
            }
            else if(front_valid) {
                ScanData.ranges[i] = front_dist;
            }
            else if(back_valid) {
                ScanData.ranges[i] = back_dist;
            }
        }
        else{
            if(theta>=front_min_angle_ && theta<=front_max_angle_){
                int scan_value = static_cast<int>((theta-scan1->angle_min)/scan1->angle_increment);
                scan_value %= 360;
                
                float raw_range = scan1->ranges[scan_value];

                // Transform to center frame
                tf2::Vector3 point(
                    raw_range * cos(theta) + front_offset_x_,
                    raw_range * sin(theta) + front_offset_y_,
                    0.0
                );
                ScanData.ranges[i] = point.length();
            }
            else if(theta>=back_min_angle_ && theta<=back_max_angle_){
                int scan_value = static_cast<int>((theta-scan2->angle_min)/scan2->angle_increment);
                scan_value %= 360;
                
                float raw_range = scan2->ranges[scan_value];

                // Transform to center frame
                tf2::Vector3 point(
                    raw_range * cos(theta) + back_offset_x_,
                    raw_range * sin(theta) + back_offset_y_,
                    0.0
                );
                ScanData.ranges[i] = point.length();
            }
        }

    
    }

    scanCombined->publish(ScanData);

    RCLCPP_INFO_ONCE(this->get_logger(),"Published Combined Scan data");
}

bool DataFuse::is_value_overlapping(double theta){
        theta = fmod(theta, 2*M_PI);
        if(theta < 0) theta += 2*M_PI;

        // Check first overlap region (front_min to back_max)
        bool overlap1 = (theta >= front_min_angle_) && (theta <= back_max_angle_);
        
        // Check second overlap region (back_min to front_max wrapped)
        bool overlap2 = (theta >= back_min_angle_) && 
                       (theta <= (front_max_angle_ - 2*M_PI));

        return overlap1 || overlap2;
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto Node = std::make_shared<DataFuse>();
    rclcpp::spin(Node);
    rclcpp::shutdown();
}