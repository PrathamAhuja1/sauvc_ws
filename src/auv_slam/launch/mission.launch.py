#!/usr/bin/env python3
"""
Gate Detection Mission Launch File
Launches only the necessary components for gate detection task
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    auv_slam_share = get_package_share_directory('auv_slam')
    
    # Config paths
    thruster_params = os.path.join(auv_slam_share, 'config', 'thruster_params.yaml')
    gate_params = os.path.join(auv_slam_share, 'config', 'gate_params.yaml')
    safety_params = os.path.join(auv_slam_share, 'config', 'safety_params.yaml')
    
    # ================================================
    # 1. SIMULATION (Gazebo + RViz)
    # ================================================
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(auv_slam_share, 'launch', 'display.launch.py')
        ),
        launch_arguments={'use_sim_time': 'True'}.items()
    )
    
    # ================================================
    # 2. THRUSTER MAPPER (Control Interface)
    # ================================================
    thruster_mapper = Node(
        package='auv_slam',
        executable='simple_thruster_mapper.py',
        name='thruster_mapper',
        output='screen',
        parameters=[thruster_params]
    )
    
    # ================================================
    # 3. GATE DETECTOR (Computer Vision)
    # ================================================
    gate_detector = Node(
        package='auv_slam',
        executable='gate_detector_node.py',
        name='gate_detector_node',
        output='screen',
        parameters=[gate_params]
    )
    
    # ================================================
    # 4. BEHAVIORAL TREE MISSION PLANNER
    # ================================================
    mission_planner = Node(
        package='auv_slam',
        executable='gate_mission_bt.py',
        name='gate_mission_bt',
        output='screen'
    )
    
    # ================================================
    # 5. SAFETY MONITOR
    # ================================================
    safety_monitor = Node(
        package='auv_slam',
        executable='safety_monitor_node.py',
        name='safety_monitor',
        output='screen',
        parameters=[safety_params]
    )
    
    # ================================================
    # 6. DIAGNOSTIC NODE (Optional - for debugging)
    # ================================================
    diagnostic = Node(
        package='auv_slam',
        executable='diagnostic_node.py',
        name='diagnostic_node',
        output='screen'
    )
    
    return LaunchDescription([
        display_launch,
        thruster_mapper,
        gate_detector,
        mission_planner,
        safety_monitor,
        diagnostic,
    ])