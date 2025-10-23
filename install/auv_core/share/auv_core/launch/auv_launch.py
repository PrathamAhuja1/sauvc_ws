import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    pkg_auv_core = get_package_share_directory('auv_core')
    ekf_config_path = os.path.join(pkg_auv_core, 'config/ekf.yaml')

    # EKF Node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path] # Use real time
    )

    # Serial Bridge Node
    serial_bridge_node = Node(
        package='auv_core',
        executable='serial_bridge_node',
        name='serial_bridge_node',
        output='screen',
        parameters=[{'serial_port': '/dev/ttyACM0', 'baud_rate': 115200}] # ADJUST PORT
    )

    # Depth Processor Node
    depth_processor_node = Node(
        package='auv_core',
        executable='depth_processor_node',
        name='depth_processor_node',
        output='screen',
        parameters=[{'depth_topic_in': '/auv/raw_depth',
                     'pose_topic_out': '/depth/pose',
                     'depth_variance': 0.01}] # TUNE VARIANCE
    )

    # Optical Flow Node
    optical_flow_node = Node(
        package='auv_core',
        executable='optical_flow_node',
        name='optical_flow_node',
        output='screen',
        parameters=[{'flow_method': 'farneback',
                     'blur_kernel_size': 15,
                     'publish_debug_image': False, # Usually false on robot
                     'height_source': 'odom'}]
        # Add remappings if needed
    )

    # Perception Node
    perception_node = Node(
        package='auv_core',
        executable='perception_node',
        name='perception_node',
        output='screen'
        # Add remappings if needed
    )

    # Mission Planner Node
    mission_planner_node = Node(
        package='auv_core',
        executable='mission_planner_node',
        name='mission_planner_node',
        output='screen'
    )

     # Control Node
    control_node = Node(
        package='auv_core',
        executable='control_node',
        name='control_node',
        output='screen'
    )

    # RViz (Optional, for monitoring)
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       name='rviz2',
       arguments=['-d', os.path.join(pkg_auv_core, 'config', 'auv_rviz_config.rviz')], # Create/Adapt an RViz config
       output='screen')

    return LaunchDescription([
        ekf_node,
        serial_bridge_node,
        depth_processor_node,
        optical_flow_node,
        perception_node,
        mission_planner_node,
        control_node,
        rviz # Optional for real robot
        # Add static TF publishers needed for imu_link, camera_link, etc.
    ])