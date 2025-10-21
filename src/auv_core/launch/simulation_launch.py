import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_auv_core = get_package_share_directory('auv_core')

    # Gazebo Sim launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r ' + os.path.join(pkg_auv_core, 'worlds', 'sauvc_pool.sdf')}.items(),
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
            'robot_description': open(os.path.join(pkg_auv_core, 'urdf', 'orca4_description.urdf')).read(),
            'use_sim_time': True
        }]
    )

    # Odometry Relay - Convert ground truth to filtered odometry for testing
    odom_relay = Node(
        package='topic_tools',
        executable='relay',
        name='odom_relay',
        arguments=['/ground_truth/odom', '/odometry/filtered'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Gate Detector Node
    gate_detector_node = Node(
        package='auv_core',
        executable='gate_detector_node',
        name='gate_detector_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'publish_debug': True,
            'min_contour_area': 500,
            'aspect_ratio_threshold': 2.0,
            'gate_width_meters': 1.5,
            'flare_min_area': 300,
            'flare_aspect_min': 3.0,
            'flare_danger_threshold': 0.3
        }]
    )

    # Gate Navigator Node
    gate_navigator_node = Node(
        package='auv_core',
        executable='gate_navigator_node',
        name='gate_navigator_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'target_depth': -1.5,
            'search_forward_speed': 0.4,
            'approach_speed': 0.5,
            'passing_speed': 0.6,
            'passing_duration': 5.0,
            'alignment_threshold': 0.2,
            'safe_distance_threshold': 3.0,
            'passing_distance_threshold': 1.5,
            'yaw_correction_gain': 0.6,
            'depth_correction_gain': 1.0,
            'flare_avoidance_gain': 0.8,
            'flare_avoidance_duration': 3.0
        }]
    )
    safety_monitor = Node(
        package='auv_core',
        executable='safety_monitor_node',
        name='safety_monitor_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'max_depth': -3.5,
            'min_depth': 0.2,
            'max_tilt_angle': 0.785,
            'watchdog_timeout': 5.0,
            'max_mission_time': 900.0
        }]
    )
    
    # Mission State Manager
    state_manager = Node(
        package='auv_core',
        executable='mission_state_manager',
        name='mission_state_manager',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Flare Detector
    flare_detector = Node(
        package='auv_core',
        executable='flare_detector_node',
        name='flare_detector_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'min_contour_area': 300,
            'min_aspect_ratio': 3.0,
            'publish_debug': True
        }]
    )


    # Simple Thruster Mapper
    simple_thruster_mapper = Node(
        package='auv_core',
        executable='simple_thruster_mapper',
        name='simple_thruster_mapper',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'max_thrust': 10.0
        }]
    )

    # RQT Image View for debugging (optional - comment out if not needed)
    debug_image_view = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='debug_viewer',
        arguments=['/gate/debug_image'],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        gazebo,
        TimerAction(period=3.0, actions=[bridge]),
        TimerAction(period=4.0, actions=[robot_state_publisher]),
        TimerAction(period=5.0, actions=[odom_relay]),
        TimerAction(period=6.0, actions=[gate_detector_node]),
        TimerAction(period=6.0, actions=[gate_navigator_node]),
        TimerAction(period=6.0, actions=[simple_thruster_mapper]),
        TimerAction(period=6.0, actions=[safety_monitor]),
        TimerAction(period=6.0, actions=[state_manager]),
        TimerAction(period=6.0, actions=[flare_detector]),
    ])