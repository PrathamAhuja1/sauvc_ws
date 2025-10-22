#!/usr/bin/env python3
"""
Complete Mission Launch - Fixed version with Behavior Tree
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_auv_core = get_package_share_directory('auv_core')

    # Parameter files
    gate_params = os.path.join(pkg_auv_core, 'config', 'gate_params.yaml')
    nav_params = os.path.join(pkg_auv_core, 'config', 'navigation_params.yaml')
    flare_params = os.path.join(pkg_auv_core, 'config', 'flare_params.yaml')
    safety_params = os.path.join(pkg_auv_core, 'config', 'safety_params.yaml')
    thruster_params = os.path.join(pkg_auv_core, 'config', 'thruster_params.yaml')
    mission_params = os.path.join(pkg_auv_core, 'config', 'mission_params.yaml')

    # Gazebo Sim launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': '-r ' + os.path.join(pkg_auv_core, 'worlds', 'sauvc_pool.sdf')
        }.items(),
    )

    # ROS <-> Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={os.path.join(pkg_auv_core, "config", "gz_bridge.yaml")}'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(os.path.join(
                pkg_auv_core, 'urdf', 'orca4_description.urdf')).read(),
            'use_sim_time': True
        }]
    )

    # Odometry Relay
    odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='odom_relay',
        arguments=['/ground_truth/odom', '/odometry/filtered'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Static TF Publishers
    static_tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=['0', '0', '0.01', '0', '0', '0', 'base_link', 'imu_link'],
        parameters=[{'use_sim_time': True}]
    )

    static_tf_base_to_forward_cam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_forward_cam',
        arguments=['0.2', '0', '0', '0', '0', '0', 'base_link', 'forward_camera_link'],
        parameters=[{'use_sim_time': True}]
    )

    # ========== PERCEPTION NODES ==========
    
    gate_detector_node = Node(
        package='auv_core',
        executable='gate_detector_node',
        name='gate_detector_node',
        output='screen',
        parameters=[
            gate_params,
            {'use_sim_time': True}
        ]
    )

    flare_detector = Node(
        package='auv_core',
        executable='flare_detector_node',
        name='flare_detector_node',
        output='screen',
        parameters=[
            flare_params,
            {'use_sim_time': True}
        ]
    )

    # ========== CONTROL & NAVIGATION ==========
    
    # Simple Thruster Mapper
    simple_thruster_mapper = Node(
        package='auv_core',
        executable='simple_thruster_mapper',
        name='simple_thruster_mapper',
        output='screen',
        parameters=[
            thruster_params,
            {'use_sim_time': True}
        ]
    )

    # ========== NEW: BEHAVIOR TREE NODE (Main Autonomous Controller) ==========
    
    behavior_tree_node = Node(
        package='auv_core',
        executable='behavior_tree_node',
        name='auv_behavior_tree_node',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'bt_tick_rate': 20.0,
                'mission_timeout': 900.0
            }
        ]
    )

    # ========== SAFETY & STATE MANAGEMENT ==========

    safety_monitor = Node(
        package='auv_core',
        executable='safety_monitor',
        name='safety_monitor',
        output='screen',
        parameters=[
            safety_params,
            {'use_sim_time': True}
        ]
    )
    
    state_manager = Node(
        package='auv_core',
        executable='mission_state_manager',
        name='mission_state_manager',
        output='screen',
        parameters=[
            mission_params,
            {'use_sim_time': True}
        ]
    )

    diagnostic_node = Node(
        package='auv_core',
        executable='diagnostic_node',
        name='diagnostic_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ========== LAUNCH SEQUENCE ==========

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='true', 
            description='Use simulation clock'
        ),
        
        # Start Gazebo
        gazebo,
        
        # Core infrastructure (staggered start)
        TimerAction(period=3.0, actions=[bridge]),
        TimerAction(period=4.0, actions=[robot_state_publisher]),
        TimerAction(period=4.5, actions=[
            static_tf_base_to_imu,
            static_tf_base_to_forward_cam,
        ]),
        TimerAction(period=5.0, actions=[odom_relay]),
        
        # Perception nodes
        TimerAction(period=6.0, actions=[
            gate_detector_node,
            flare_detector,
        ]),
        
        # Control layer
        TimerAction(period=7.0, actions=[
            simple_thruster_mapper
        ]),
        
        # Safety & diagnostics
        TimerAction(period=7.5, actions=[
            safety_monitor,
            state_manager,
            diagnostic_node
        ]),
        
        # ========== START AUTONOMOUS MISSION ==========
        TimerAction(period=8.0, actions=[
            behavior_tree_node  # This is the main controller!
        ]),
    ])