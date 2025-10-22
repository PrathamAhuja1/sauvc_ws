#!/usr/bin/env python3


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

    # Odometry Relay (for testing without full EKF)
    odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='odom_relay',
        arguments=['/ground_truth/odom', '/odometry/filtered'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ========== PERCEPTION NODES ==========
    
    # Gate Detector Node (with parameters)
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

    # Flare Detector (with parameters)
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

    # Acoustic Pinger Detector
    acoustic_detector = Node(
        package='auv_core',
        executable='acoustic_pinger_detector',
        name='acoustic_pinger_detector',
        output='screen',
        parameters=[
            mission_params,
            {'use_sim_time': True}
        ]
    )

    # ========== CONTROL & NAVIGATION NODES ==========

    # Gate Navigator Node (with parameters)
    gate_navigator_node = Node(
        package='auv_core',
        executable='gate_navigator_node',
        name='gate_navigator_node',
        output='screen',
        parameters=[
            nav_params,
            {'use_sim_time': True}
        ]
    )

    # Simple Thruster Mapper (with parameters)
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

    # ========== SAFETY & STATE MANAGEMENT ==========

    # Enhanced Safety Monitor (with parameters)
    safety_monitor = Node(
        package='auv_core',
        executable='enhanced_safety_monitor',
        name='enhanced_safety_monitor',
        output='screen',
        parameters=[
            safety_params,
            {'use_sim_time': True}
        ]
    )
    
    # Enhanced Mission State Manager
    state_manager = Node(
        package='auv_core',
        executable='mission_state_manager_enhanced',
        name='mission_state_manager_enhanced',
        output='screen',
        parameters=[
            mission_params,
            {'use_sim_time': True}
        ]
    )

    # TF Tree Manager
    tf_manager = Node(
        package='auv_core',
        executable='tf_tree_manager',
        name='tf_tree_manager',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Diagnostic Node
    diagnostic_node = Node(
        package='auv_core',
        executable='diagnostic_node',
        name='diagnostic_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ========== STATIC TF PUBLISHERS ==========

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

    # ========== LAUNCH SEQUENCE ==========

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', 
            default_value='true', 
            description='Use simulation clock'
        ),
        
        DeclareLaunchArgument(
            'params_file',
            default_value=mission_params,
            description='Path to mission parameters file'
        ),
        
        # Start Gazebo
        gazebo,
        
        # Core infrastructure (staggered start)
        TimerAction(period=3.0, actions=[bridge]),
        TimerAction(period=4.0, actions=[robot_state_publisher]),
        TimerAction(period=4.5, actions=[
            static_tf_base_to_imu,
            static_tf_base_to_forward_cam,
            tf_manager
        ]),
        TimerAction(period=5.0, actions=[odom_relay]),
        
        # Perception nodes
        TimerAction(period=6.0, actions=[
            gate_detector_node,
            flare_detector,
            acoustic_detector
        ]),
        
        # Control & navigation
        TimerAction(period=7.0, actions=[
            gate_navigator_node,
            simple_thruster_mapper
        ]),
        
        # Safety & state management
        TimerAction(period=7.5, actions=[
            safety_monitor,
            state_manager,
            diagnostic_node
        ]),
    ])