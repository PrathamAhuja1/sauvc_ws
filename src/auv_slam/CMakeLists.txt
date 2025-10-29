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

# Find Pangolin (required by ORB_SLAM3)
find_package(Pangolin REQUIRED)

find_path(ORB_SLAM3_INCLUDE_DIR
    NAMES System.h
    PATHS
        ~/Documents/ORB_SLAM3/include
        /home/pratham/Documents/ORB_SLAM3/include
        /usr/local/include/ORB_SLAM3
        $ENV{ORB_SLAM3_ROOT}/include
    PATH_SUFFIXES ORB_SLAM3
)

install(
  DIRECTORY src launch rviz worlds models config
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES src/description/orca4_description.urdf
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(PROGRAMS
  scripts/bluerov_teleop_ign.py
  scripts/gate_detector_node.py
  scripts/gate_navigator_node.py
  scripts/flare_detection.py
  scripts/safety_monitor_node.py
  scripts/diagnostic_node.py
  scripts/simple_thruster_mapper.py
  scripts/BT.py
  scripts/drum_detector_node.py 
  scripts/drum_navigator_node.py  
  scripts/flare_navigator_node.py 
  DESTINATION lib/${PROJECT_NAME}
)

# Find ORB_SLAM3 library
find_library(ORB_SLAM3_LIBRARY
    NAMES ORB_SLAM3
    PATHS
        ~/Documents/ORB_SLAM3/lib
        /home/pratham/Documents/ORB_SLAM3/lib
        /usr/local/lib
        $ENV{ORB_SLAM3_ROOT}/lib
)

# Find DBoW2 headers
find_path(DBoW2_INCLUDE_DIR
    NAMES BowVector.h
    PATHS
        /usr/local/include/DBoW2
        /usr/local/include
        ~/Documents/ORB_SLAM3/Thirdparty/DBoW2/DBoW2
        /home/pratham/Documents/ORB_SLAM3/Thirdparty/DBoW2/DBoW2
        ~/Documents/ORB_SLAM3/Thirdparty/DBoW2
        /home/pratham/Documents/ORB_SLAM3/Thirdparty/DBoW2
)

# Find DUtils headers (also needed by DBoW2)
find_path(DUtils_INCLUDE_DIR
    NAMES Timestamp.h
    PATHS
        /usr/local/include/DUtils
        /usr/local/include
        ~/Documents/ORB_SLAM3/Thirdparty/DBoW2/DUtils
        /home/pratham/Documents/ORB_SLAM3/Thirdparty/DBoW2/DUtils
        ~/Documents/ORB_SLAM3/Thirdparty/DBoW2
        /home/pratham/Documents/ORB_SLAM3/Thirdparty/DBoW2
)

# Find DBoW2 library
find_library(DBoW2_LIBRARY
    NAMES DBoW2 libDBoW2.so libDBoW2.a
    PATHS
        /usr/local/lib
        ~/Documents/ORB_SLAM3/Thirdparty/DBoW2/lib
        $ENV{ORB_SLAM3_ROOT}/Thirdparty/DBoW2/lib
        /home/pratham/Documents/ORB_SLAM3/Thirdparty/DBoW2/lib
        ~/Documents/ORB_SLAM3/lib
        /home/pratham/Documents/ORB_SLAM3/lib
)

# Find g2o library (single library instead of multiple components)
find_library(g2o_LIBRARY
    NAMES g2o libg2o.so libg2o.a
    PATHS
        /usr/local/lib
        ~/Documents/ORB_SLAM3/Thirdparty/g2o/lib
        /home/pratham/Documents/ORB_SLAM3/Thirdparty/g2o/lib
        ~/Documents/ORB_SLAM3/lib
        /home/pratham/Documents/ORB_SLAM3/lib
)

# Set g2o_LIBRARIES to the single library we found
set(g2o_LIBRARIES ${g2o_LIBRARY})

# Find Sophus headers
find_path(Sophus_INCLUDE_DIR
    NAMES sophus/se3.hpp
    PATHS
        /usr/local/include
        /usr/include
        ~/Documents/ORB_SLAM3/Thirdparty/Sophus
        $ENV{ORB_SLAM3_ROOT}/Thirdparty/Sophus
        /home/pratham/Documents/ORB_SLAM3/Thirdparty/Sophus
)

# Check if all required components are found
if(NOT ORB_SLAM3_INCLUDE_DIR OR NOT ORB_SLAM3_LIBRARY OR NOT DBoW2_INCLUDE_DIR OR NOT DBoW2_LIBRARY OR NOT g2o_LIBRARY)
    message(FATAL_ERROR "Could NOT find ORB_SLAM3 components. Found: ORB_SLAM3_INCLUDE_DIR=${ORB_SLAM3_INCLUDE_DIR}, ORB_SLAM3_LIBRARY=${ORB_SLAM3_LIBRARY}, DBoW2_INCLUDE_DIR=${DBoW2_INCLUDE_DIR}, DBoW2_LIBRARY=${DBoW2_LIBRARY}, g2o_LIBRARY=${g2o_LIBRARY}")
endif()

# Set include directories and libraries
set(ORB_SLAM3_INCLUDE_DIRS
    ${ORB_SLAM3_INCLUDE_DIR}
    ${ORB_SLAM3_INCLUDE_DIR}/CameraModels
    ${DBoW2_INCLUDE_DIR}
    ${DUtils_INCLUDE_DIR}
    ${Sophus_INCLUDE_DIR}
    # Add the parent directory so that "Thirdparty/DBoW2/DBoW2/BowVector.h" resolves correctly
    /home/pratham/Documents/ORB_SLAM3
    ~/Documents/ORB_SLAM3
)

set(ORB_SLAM3_LIBRARIES
    ${ORB_SLAM3_LIBRARY}
    ${DBoW2_LIBRARY}
    ${g2o_LIBRARIES}
)

message(STATUS "Found ORB_SLAM3: ${ORB_SLAM3_LIBRARY}")
message(STATUS "ORB_SLAM3 include dirs: ${ORB_SLAM3_INCLUDE_DIRS}")
# ===== End ORB_SLAM3 finding =====

# Create the library/executable
add_executable(stereo_slam_node src/stereo_slam_node.cpp)

# Include directories (ORB_SLAM3 first to prioritize its Sophus)
target_include_directories(stereo_slam_node PRIVATE
    ${ORB_SLAM3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
)

# Exclude system Sophus to avoid conflicts
target_include_directories(stereo_slam_node SYSTEM BEFORE PRIVATE
    /home/pratham/Documents/ORB_SLAM3/Thirdparty/Sophus
)

# Link ROS 2 libraries
ament_target_dependencies(stereo_slam_node
  rclcpp
  sensor_msgs
  geometry_msgs
  nav_msgs
  cv_bridge
  message_filters
)

# Link ORB_SLAM3 and other libraries (including Pangolin)
target_link_libraries(stereo_slam_node
    ${ORB_SLAM3_LIBRARIES}
    ${Pangolin_LIBRARIES}
    ${OpenCV_LIBS}
    pthread
    GL
    GLEW
)

# Keep Pangolin enabled since it's now installed
# target_compile_definitions(stereo_slam_node PRIVATE
#     -DCOMPILED_WITHOUT_PANGOLIN
# )

# Install the executable
install(TARGETS
  stereo_slam_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files if they exist
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

ament_package()
