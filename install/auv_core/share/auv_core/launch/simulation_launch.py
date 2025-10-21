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

    # Gazebo Sim launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        # Use -r to start simulation paused. Remove '-r' to start unpaused.
        launch_arguments={'gz_args': '-r ' + os.path.join(pkg_auv_core, 'worlds', 'sauvc_pool.sdf')}.items(),
    )

    # Spawn robot entity (Loads URDF and adds Gazebo plugins)
    spawn = Node(package='ros_gz_sim', executable='create', # Use 'create' from ros_gz_sim
                 arguments=['-name', 'orca4', # Must match name in bridge config/topics if using '/model/...'
                            '-file', os.path.join(pkg_auv_core, 'urdf', 'orca4_description.urdf'),
                            '-x', '0',
                            '-y', '0',
                            '-z', '0.1', # Start slightly above water surface
                            '-Y', '0'], # Yaw
                 output='screen')

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
        parameters=[{'robot_description': open(os.path.join(pkg_auv_core, 'urdf', 'orca4_description.urdf')).read(),
                     'use_sim_time': True}])

    # --- Start AUV Core Nodes ---
    # EKF Node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_auv_core, 'config/ekf.yaml'), {'use_sim_time': True}]
    )

    # Depth Processor Node (using ground truth odometry from bridge)
    depth_processor_node = Node(
        package='auv_core',
        executable='depth_processor_node',
        name='depth_processor_node',
        output='screen',
        parameters=[{'sim_odom_topic_in': '/ground_truth/odom',
                     'pose_topic_out': '/depth/pose',
                     'depth_variance': 0.0001,
                     'use_sim_time': True}]
    )

    # Optical Flow Node
    optical_flow_node = Node(
        package='auv_core',
        executable='optical_flow_node',
        name='optical_flow_node',
        output='screen',
        parameters=[{'flow_method': 'farneback',
                     'blur_kernel_size': 5,
                     'publish_debug_image': True,
                     'height_source': 'odom', # Use EKF odom for height
                     'use_sim_time': True}],
        remappings=[('/camera/image_raw', '/camera_down/image_raw'), # Ensure topics match bridge/URDF
                    ('/camera/camera_info', '/camera_down/camera_info'),
                    ('/odometry/filtered', '/odometry/filtered')]
    )

    # --- AUTONOMOUS GATE NAVIGATION NODES ---

    # Gate Detector Node
    gate_detector_node = Node(
        package='auv_core',
        executable='gate_detector_node', # Must be in setup.py entry_points
        name='gate_detector_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Gate Navigator Node
    gate_navigator_node = Node(
        package='auv_core',
        executable='gate_navigator_node', # Must be in setup.py entry_points
        name='gate_navigator_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            # Remap output Twist topic to match sim_thruster_mapper's input
            ('/rp2040/cmd_vel', '/control/sim_cmd_vel')
        ]
    )

    # Sim Thruster Mapper (Converts Twist to Float64 for bridge)
    # This node now listens to the remapped topic from gate_navigator_node
    sim_thruster_mapper = Node(
        package='auv_core',
        executable='sim_thruster_mapper', # Make sure setup.py has this entry point
        name='sim_thruster_mapper',
        output='screen',
        parameters=[{'cmd_vel_topic_in': '/control/sim_cmd_vel', # Input Twist topic
                     'num_thrusters': 6,
                     'use_sim_time': True}],
        # This node publishes to /thruster1_cmd, /thruster2_cmd etc.
    )

    # --- MANUAL CONTROL NODES (COMMENTED OUT) ---
    # Control Node (Placeholder - publishes Twist)
    # control_node = Node(
    #     package='auv_core',
    #     executable='control_node', # Make sure setup.py has this entry point
    #     name='control_node',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}],
    #     # This node should publish to '/control/sim_cmd_vel'
    # )

    # Teleop node for testing control
    # teleop_node = Node(
    #     package='auv_core',
    #     executable='bluerov_teleop', # Matches entry point in setup.py
    #     name='auv_teleop',
    #     output='screen',
    #     prefix="xterm -e", # Launch in a separate terminal window
    #     parameters=[{'use_sim_time': True}],
    #     remappings=[ # Remap output topics to match sim_thruster_mapper's PUBLISHED topics
    #          ('/model/orca4_ign/joint/thruster1_joint/cmd_pos', '/thruster1_cmd'),
    #          ('/model/orca4_ign/joint/thruster2_joint/cmd_pos', '/thruster2_cmd'),
    #          ('/model/orca4_ign/joint/thruster3_joint/cmd_pos', '/thruster3_cmd'),
    #          ('/model/orca4_ign/joint/thruster4_joint/cmd_pos', '/thruster4_cmd'),
    #          ('/model/orca4_ign/joint/thruster5_joint/cmd_pos', '/thruster5_cmd'),
    #          ('/model/orca4_ign/joint/thruster6_joint/cmd_pos', '/thruster6_cmd'),
    #     ]
    # )
    # --- END MANUAL CONTROL ---


    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock'),
        gazebo,
        # Delay spawn and bridge until Gazebo server is up
        TimerAction(period=5.0, actions=[spawn]),
        TimerAction(period=7.0, actions=[bridge]),
        TimerAction(period=8.0, actions=[robot_state_publisher]),
        TimerAction(period=9.0, actions=[ekf_node]),
        TimerAction(period=9.0, actions=[depth_processor_node]),
        TimerAction(period=9.0, actions=[optical_flow_node]),
        TimerAction(period=9.0, actions=[sim_thruster_mapper]),

        # Removed manual control nodes from launch timers
        # TimerAction(period=9.0, actions=[control_node]),
        # TimerAction(period=10.0, actions=[teleop_node]), # Launch teleop last

        # Add autonomous nodes to launch
        TimerAction(period=10.0, actions=[gate_detector_node]),
        TimerAction(period=10.0, actions=[gate_navigator_node]),
    ])