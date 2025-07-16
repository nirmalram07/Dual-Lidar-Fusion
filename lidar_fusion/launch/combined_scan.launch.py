from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_dir = get_package_share_directory("lidar_fusion")

    lidar_pose = os.path.join(package_dir, "combinedscan_params.yaml")

    combined_scan_node = Node(
        package="lidar_fusion",
        executable="combined_scan",
        parameters=[lidar_pose]
    )

    return LaunchDescription([
        combined_scan_node
    ])