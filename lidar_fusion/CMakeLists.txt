cmake_minimum_required(VERSION 3.8)
project(lidar_fusion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(combined_scan src/dataFuseNode.cpp)
target_include_directories(combined_scan PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(combined_scan rclcpp sensor_msgs tf2 tf2_ros geometry_msgs)

install(TARGETS
  combined_scan
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch/ params/
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp sensor_msgs tf2 tf2_ros geometry_msgs)

ament_package()

