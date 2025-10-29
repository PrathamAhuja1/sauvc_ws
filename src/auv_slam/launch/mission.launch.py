#!/usr/bin/env python3
"""
Complete Autonomous Mission Launch File (Professional State Machine Version)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    auv_slam_share = get_package_share_directory('auv_slam')
    
    # This single param file is used by both nodes
    param_file = os.path.join(auv_slam_share, 'config', 'gate_params.yaml')
    
    # Config for thrusters
    thruster_params = os.path.join(auv_slam_share, 'config', 'thruster_params.yaml')
    
    # 1. Simulation (Gazebo + RViz)
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(auv_slam_share, 'launch', 'display.launch.py')
        ),
        launch_arguments={'use_sim_time': 'True'}.items()
    )

    # Gazebo-ROS Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p', f'config_file:={os.path.join(auv_slam_share, "config", "gz_bridge.yaml")}'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 2. Fixed Thruster Mapper
    thruster_mapper = Node(
        package='auv_slam',
        executable='simple_thruster_mapper.py',
        name='fixed_thruster_mapper',
        output='screen',
        parameters=[thruster_params]
    )
    
    # 3. Gate Detector (Updated)
    gate_detector = Node(
        package='auv_slam',
        executable='gate_detector_node.py',
        name='gate_detector_node',
        output='screen',
        parameters=[param_file] # Loads params from gate_detector_node section
    )

    # 4. Professional Gate Navigator (NEW)
    gate_navigator = Node(
        package='auv_slam',
        executable='gate_navigator_node.py',
        name='gate_navigator_node', # This MUST match the name in the YAML
        output='screen',
        parameters=[param_file] # Loads params from professional_gate_navigator section
    )
    
    # 5. Safety Monitor
    safety_monitor = Node(
        package='auv_slam',
        executable='safety_monitor_node.py',
        name='safety_monitor',
        output='screen'
    )
    
    # 6. Diagnostic Node
    diagnostic = Node(
       package='auv_slam',
        executable='diagnostic_node.py',
        name='diagnostic_node',
        output='screen'
    )
    
    return LaunchDescription([
        display_launch,
        bridge,
        thruster_mapper,
        gate_detector,
        gate_navigator,  # <-- This is the new professional navigator
        safety_monitor,
        diagnostic,
    ])