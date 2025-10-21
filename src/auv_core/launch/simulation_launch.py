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

    # --- AUV Core Nodes ---
    
    # Gate Detector Node
    gate_detector_node = Node(
        package='auv_core',
        executable='gate_detector_node',
        name='gate_detector_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'publish_debug': True,
            'min_contour_area': 800,
            'aspect_ratio_threshold': 2.5,
            'gate_width_meters': 1.5
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
            'search_forward_speed': 0.3,
            'approach_speed': 0.4,
            'passing_speed': 0.5,
            'alignment_threshold': 0.15,
            'safe_distance_threshold': 2.5,
            'passing_distance_threshold': 1.0,
            'yaw_correction_gain': 0.5,
            'depth_correction_gain': 0.8,
            'flare_avoidance_gain': 0.7
        }]
    )

    # Simple Thruster Mapper
    simple_thruster_mapper = Node(
        package='auv_core',
        executable='simple_thruster_mapper',
        name='simple_thruster_mapper',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }]
    )

    # Image visualization (optional - for debugging)
    image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='camera_viewer',
        arguments=['/camera_down/image_raw'],
        parameters=[{'use_sim_time': True}]
    )

    # Debug image viewer (optional)
    debug_image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='debug_viewer',
        arguments=['/gate/debug_image'],
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock'),
        gazebo,
        # Robot spawns from world file, so bridge and other nodes can start sooner
        TimerAction(period=5.0, actions=[bridge]),
        TimerAction(period=6.0, actions=[robot_state_publisher]),
        TimerAction(period=7.0, actions=[gate_detector_node]),
        TimerAction(period=7.0, actions=[gate_navigator_node]),
        TimerAction(period=7.0, actions=[simple_thruster_mapper]),
        # Uncomment these to view camera feeds
        # TimerAction(period=8.0, actions=[image_view_node]),
        # TimerAction(period=8.0, actions=[debug_image_view_node]),
    ])