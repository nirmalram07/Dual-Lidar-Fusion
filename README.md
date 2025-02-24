# Dual Lidar Fusion for Combined Laser Scans in Mobile Robots

## Overview
This project demonstrates a dual lidar fusion system on a mobile robot. The robot features two lidar sensors—one mounted at the front left bottom and another at the back right bottom—integrated via a custom URDF in a single package. A separate package hosts an rclcpp node that:
- Subscribes to both individual lidar scan topics.
- Creates a dummy TF frame at the center of the robot.
- Publishes a combined laser scan relative to that dummy TF frame.

Future plans include integrating this system with the ROS2 navigation stack for autonomous operation.

## Features
- **Dual Lidar Integration:** Combines data from two lidars mounted at different positions on the robot.
- **Custom URDF:** Defines the robot model with both lidar sensors.
- **Combined Laser Scan Publisher:** A ROS2 node subscribes to both lidar topics and publishes a fused laser scan.
- **Development Branch:** The latest code is in the `development` branch, where active development is ongoing.
- **Known Issues:**
  - Mismatch between individual lidar scan data and the combined scan.
  - Flickering in the intersection area of the two lidar scans.

## Requirements
- **ROS2:** A supported ROS2 distribution (e.g., Foxy, Galactic, Humble) must be installed and configured.
- **C++:** C++17 or later.
- **Additional Dependencies:** Please refer to the package's xml files for any additional libraries or tools required.


## Installation
1. **Clone the Repository:**
   
   ```bash
   mkdir lidar_fusion_ws
   cd lidar_fusion_ws
   mkdir src
   cd src
   git clone https://github.com/nirmalram07/Warehouse-AMR-Behavior-Tree.git

2. **Build the Packages:**
   
   ```bash
   cd ../
   colcon build --symlink-install
   source install/setup.bash
   
3. **Launching the Nodes:**
   
      ```bash
      source install/setup.bash 
      ros2 launch lidar_fuse_robot diff_robot.launch.py
      ```
   Now open a new terminal tab to launch the fusion node
   
      ```bash
      source install/setup.bash
      ros2 run lidar_fusion combined_scan
      ```

## Suggestions & Feedbacks

Contributions are welcome! If you have suggestions, bug fixes, or improvements, please open an issue or submit a pull request.

