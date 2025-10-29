cmake_minimum_required(VERSION 3.8)
project(auv_slam)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

# Find OpenCV and Eigen3
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Install launch, config, and world files
install(
  DIRECTORY launch rviz worlds models config
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES src/description/orca4_description.urdf
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install Python scripts - GATE DETECTION ONLY
install(PROGRAMS
  # Manual control (for testing)
  scripts/bluerov_teleop_ign.py
  
  # Core gate detection system
  scripts/gate_detector_node.py
  scripts/gate_mission_bt.py
  
  # Support systems
  scripts/simple_thruster_mapper.py
  scripts/safety_monitor_node.py
  scripts/diagnostic_node.py
  
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()