#!/usr/bin/env python3
"""
Main Behavior Tree for SAUVC Tasks 1 (Navigation) and 4 (Communication & Localization)
Uses py_trees library for behavior tree implementation
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import py_trees
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees import blackboard
from std_msgs.msg import Bool, Float32, String
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
import time
import math


class MissionSetup(Behaviour):
    """Initialize mission parameters and check prerequisites"""
    
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.bb = blackboard.Client(name=name)
        
    def setup(self, **kwargs):
        self.bb.register_key(key='mission_start_time', access=py_trees.common.Access.WRITE)
        self.bb.register_key(key='current_depth', access=py_trees.common.Access.WRITE)
        self.bb.register_key(key='target_depth', access=py_trees.common.Access.WRITE)
        self.bb.register_key(key='tasks_completed', access=py_trees.common.Access.WRITE)
        self.bb.register_key(key='flare_sequence', access=py_trees.common.Access.WRITE)
        self.bb.register_key(key='navigation_complete', access=py_trees.common.Access.WRITE)
        self.bb.register_key(key='emergency_stop', access=py_trees.common.Access.WRITE)
        
    def initialise(self):
        self.node.get_logger().info("Initializing mission setup...")
        
    def update(self):
        # Set initial values
        self.bb.mission_start_time = time.time()
        self.bb.current_depth = 0.0
        self.bb.target_depth = -1.5
        self.bb.tasks_completed = []
        self.bb.navigation_complete = False
        self.bb.emergency_stop = False
        self.bb.flare_sequence = []
        
        self.node.get_logger().info("Mission setup complete!")
        return Status.SUCCESS


class SubmergeToDepth(Behaviour):
    """Submerge AUV to target depth"""
    
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.bb = blackboard.Client(name=name)
        self.cmd_vel_pub = node.create_publisher(Twist, '/rp2040/cmd_vel', 10)
        self.depth_tolerance = 0.2
        
    def setup(self, **kwargs):
        self.bb.register_key(key='current_depth', access=py_trees.common.Access.READ)
        self.bb.register_key(key='target_depth', access=py_trees.common.Access.READ)
        
    def initialise(self):
        self.node.get_logger().info(f"Submerging to depth: {self.bb.target_depth}m")
        
    def update(self):
        depth_error = self.bb.target_depth - self.bb.current_depth
        
        if abs(depth_error) < self.depth_tolerance:
            # Stop vertical motion
            cmd = Twist()
            cmd.linear.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.node.get_logger().info(f"Target depth reached: {self.bb.current_depth:.2f}m")
            return Status.SUCCESS
        
        # Apply depth correction
        cmd = Twist()
        cmd.linear.z = depth_error * 0.8
        self.cmd_vel_pub.publish(cmd)
        
        return Status.RUNNING
    
    def terminate(self, new_status):
        # Stop motion on termination
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)


class CheckGateDetected(Behaviour):
    """Check if navigation gate is detected"""
    
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.bb = blackboard.Client(name=name)
        self.gate_detected = False
        self.last_detection_time = None
        self.detection_timeout = 2.0
        
        self.gate_sub = node.create_subscription(
            Bool, '/gate/detected', self.gate_callback, 10)
        
    def setup(self, **kwargs):
        self.bb.register_key(key='gate_detected', access=py_trees.common.Access.WRITE)
        
    def gate_callback(self, msg):
        self.gate_detected = msg.data
        if self.gate_detected:
            self.last_detection_time = time.time()
    
    def update(self):
        # Check if detection is recent
        if self.last_detection_time:
            time_since_detection = time.time() - self.last_detection_time
            if time_since_detection < self.detection_timeout:
                self.bb.gate_detected = True
                return Status.SUCCESS
        
        self.bb.gate_detected = False
        return Status.FAILURE


class SearchForGate(Behaviour):
    """Search pattern to find the navigation gate"""
    
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.bb = blackboard.Client(name=name)
        self.cmd_vel_pub = node.create_publisher(Twist, '/rp2040/cmd_vel', 10)
        self.search_start_time = None
        self.search_timeout = 30.0
        
    def initialise(self):
        self.search_start_time = time.time()
        self.node.get_logger().info("Starting gate search pattern...")
        
    def update(self):
        # Check timeout
        if time.time() - self.search_start_time > self.search_timeout:
            self.node.get_logger().warn("Gate search timeout!")
            return Status.FAILURE
        
        # Simple search: move forward slowly
        cmd = Twist()
        cmd.linear.x = 0.3
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        
        return Status.RUNNING
    
    def terminate(self, new_status):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)


class AlignToGate(Behaviour):
    """Align AUV to the center of the gate"""
    
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.bb = blackboard.Client(name=name)
        self.cmd_vel_pub = node.create_publisher(Twist, '/rp2040/cmd_vel', 10)
        self.alignment_error = 0.0
        self.alignment_tolerance = 0.15
        
        self.alignment_sub = node.create_subscription(
            Float32, '/gate/alignment_error', self.alignment_callback, 10)
        
    def alignment_callback(self, msg):
        self.alignment_error = msg.data
    
    def initialise(self):
        self.node.get_logger().info("Aligning to gate...")
        
    def update(self):
        if abs(self.alignment_error) < self.alignment_tolerance:
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            self.node.get_logger().info("Gate alignment complete!")
            return Status.SUCCESS
        
        # Apply yaw correction
        cmd = Twist()
        cmd.linear.x = 0.1  # Slow forward
        cmd.angular.z = -self.alignment_error * 0.5
        self.cmd_vel_pub.publish(cmd)
        
        return Status.RUNNING
    
    def terminate(self, new_status):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)


class PassThroughGate(Behaviour):
    """Pass through the navigation gate"""
    
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.bb = blackboard.Client(name=name)
        self.cmd_vel_pub = node.create_publisher(Twist, '/rp2040/cmd_vel', 10)
        self.gate_distance = 100.0
        self.pass_start_time = None
        self.pass_duration = 5.0
        
        self.distance_sub = node.create_subscription(
            Float32, '/gate/estimated_distance', self.distance_callback, 10)
        
    def setup(self, **kwargs):
        self.bb.register_key(key='navigation_complete', access=py_trees.common.Access.WRITE)
        
    def distance_callback(self, msg):
        self.gate_distance = msg.data
    
    def initialise(self):
        self.pass_start_time = time.time()
        self.node.get_logger().info("Passing through gate...")
        
    def update(self):
        elapsed = time.time() - self.pass_start_time
        
        if elapsed > self.pass_duration:
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)
            self.bb.navigation_complete = True
            self.node.get_logger().info("Gate passage complete!")
            return Status.SUCCESS
        
        # Full speed ahead
        cmd = Twist()
        cmd.linear.x = 0.5
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        
        return Status.RUNNING
    
    def terminate(self, new_status):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)


class WaitForFlareSequence(Behaviour):
    """Wait for flare sequence communication from surface"""
    
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.bb = blackboard.Client(name=name)
        self.sequence_received = False
        self.flare_sequence = []
        self.wait_start_time = None
        self.wait_timeout = 10.0
        
        # Subscribe to communication topic
        self.comm_sub = node.create_subscription(
            String, '/auv/flare_sequence', self.sequence_callback, 10)
        
    def setup(self, **kwargs):
        self.bb.register_key(key='flare_sequence', access=py_trees.common.Access.WRITE)
        
    def sequence_callback(self, msg):
        # Parse sequence like "R-B-Y"
        sequence_str = msg.data.strip()
        self.flare_sequence = sequence_str.split('-')
        self.sequence_received = True
        self.node.get_logger().info(f"Received flare sequence: {sequence_str}")
    
    def initialise(self):
        self.wait_start_time = time.time()
        self.sequence_received = False
        self.node.get_logger().info("Waiting for flare sequence...")
        
    def update(self):
        if self.sequence_received:
            self.bb.flare_sequence = self.flare_sequence
            return Status.SUCCESS
        
        if time.time() - self.wait_start_time > self.wait_timeout:
            self.node.get_logger().warn("Flare sequence timeout, using default: R-Y-B")
            self.bb.flare_sequence = ['R', 'Y', 'B']
            return Status.SUCCESS
        
        return Status.RUNNING


class SearchForFlare(Behaviour):
    """Search for a specific colored flare"""
    
    def __init__(self, name, node, target_color):
        super().__init__(name)
        self.node = node
        self.bb = blackboard.Client(name=name)
        self.target_color = target_color
        self.flare_detected = False
        self.flare_position = None
        self.search_start_time = None
        self.search_timeout = 45.0
        self.cmd_vel_pub = node.create_publisher(Twist, '/rp2040/cmd_vel', 10)
        
        # Subscribe to flare detection topics based on color
        color_topics = {
            'R': '/flare/red_detected',
            'Y': '/flare/yellow_detected',
            'B': '/flare/blue_detected'
        }
        
        if target_color in color_topics:
            self.flare_sub = node.create_subscription(
                PoseStamped, color_topics[target_color], 
                self.flare_callback, 10)
    
    def flare_callback(self, msg):
        self.flare_detected = True
        self.flare_position = msg.pose.position
    
    def initialise(self):
        self.search_start_time = time.time()
        self.flare_detected = False
        color_names = {'R': 'Red', 'Y': 'Yellow', 'B': 'Blue'}
        self.node.get_logger().info(
            f"Searching for {color_names.get(self.target_color, 'Unknown')} flare...")
        
    def update(self):
        if self.flare_detected:
            return Status.SUCCESS
        
        if time.time() - self.search_start_time > self.search_timeout:
            self.node.get_logger().warn(f"Flare {self.target_color} search timeout!")
            return Status.FAILURE
        
        # Search pattern: sweep motion
        cmd = Twist()
        cmd.linear.x = 0.3
        cmd.angular.z = 0.2 * math.sin((time.time() - self.search_start_time) * 0.5)
        self.cmd_vel_pub.publish(cmd)
        
        return Status.RUNNING
    
    def terminate(self, new_status):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)


class ApproachFlare(Behaviour):
    """Approach and bump the flare"""
    
    def __init__(self, name, node, target_color):
        super().__init__(name)
        self.node = node
        self.target_color = target_color
        self.cmd_vel_pub = node.create_publisher(Twist, '/rp2040/cmd_vel', 10)
        self.flare_position = None
        self.approach_start_time = None
        self.approach_duration = 8.0
        
        color_topics = {
            'R': '/flare/red_detected',
            'Y': '/flare/yellow_detected',
            'B': '/flare/blue_detected'
        }
        
        if target_color in color_topics:
            self.flare_sub = node.create_subscription(
                PoseStamped, color_topics[target_color], 
                self.flare_callback, 10)
    
    def flare_callback(self, msg):
        self.flare_position = msg.pose.position
    
    def initialise(self):
        self.approach_start_time = time.time()
        color_names = {'R': 'Red', 'Y': 'Yellow', 'B': 'Blue'}
        self.node.get_logger().info(
            f"Approaching {color_names.get(self.target_color, 'Unknown')} flare...")
        
    def update(self):
        elapsed = time.time() - self.approach_start_time
        
        if elapsed > self.approach_duration:
            self.node.get_logger().info(f"Flare {self.target_color} bumped!")
            return Status.SUCCESS
        
        # Move toward flare
        cmd = Twist()
        if self.flare_position:
            # Simple proportional control toward flare
            lateral_error = self.flare_position.x / 400.0  # Normalize pixel position
            cmd.angular.z = -lateral_error * 0.4
        
        cmd.linear.x = 0.4
        self.cmd_vel_pub.publish(cmd)
        
        return Status.RUNNING
    
    def terminate(self, new_status):
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)


class EmergencyStop(Behaviour):
    """Emergency stop - surface immediately"""
    
    def __init__(self, name, node):
        super().__init__(name)
        self.node = node
        self.cmd_vel_pub = node.create_publisher(Twist, '/rp2040/cmd_vel', 10)
        
    def update(self):
        self.node.get_logger().error("EMERGENCY STOP ACTIVATED!")
        cmd = Twist()
        cmd.linear.z = -1.0  # Surface
        self.cmd_vel_pub.publish(cmd)
        return Status.SUCCESS


class CheckMissionTimeout(Behaviour):
    """Check if mission has exceeded time limit"""
    
    def __init__(self, name, node, timeout=900.0):
        super().__init__(name)
        self.node = node
        self.bb = blackboard.Client(name=name)
        self.timeout = timeout
        
    def setup(self, **kwargs):
        self.bb.register_key(key='mission_start_time', access=py_trees.common.Access.READ)
        self.bb.register_key(key='emergency_stop', access=py_trees.common.Access.WRITE)
        
    def update(self):
        elapsed = time.time() - self.bb.mission_start_time
        
        if elapsed > self.timeout:
            self.node.get_logger().error(f"Mission timeout! ({elapsed:.1f}s)")
            self.bb.emergency_stop = True
            return Status.FAILURE
        
        return Status.SUCCESS


class AUVBehaviorTreeNode(Node):
    """Main ROS2 node that runs the behavior tree"""
    
    def __init__(self):
        super().__init__('auv_behavior_tree_node')
        
        # Declare parameters
        self.declare_parameter('bt_tick_rate', 20.0)
        self.declare_parameter('mission_timeout', 900.0)
        
        self.tick_rate = self.get_parameter('bt_tick_rate').value
        self.mission_timeout = self.get_parameter('mission_timeout').value
        
        # Subscribe to odometry for depth
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)
        self.current_depth = 0.0
        
        # Build behavior tree
        self.root = self.create_behavior_tree()
        
        # Setup tree
        self.root.setup_with_descendants()
        
        # Create timer for ticking
        self.timer = self.create_timer(
            1.0 / self.tick_rate, self.tick_tree)
        
        self.get_logger().info("AUV Behavior Tree Node started!")
        
    def odom_callback(self, msg):
        self.current_depth = msg.pose.pose.position.z
        bb = blackboard.Client(name="odom")
        bb.register_key(key='current_depth', access=py_trees.common.Access.WRITE)
        bb.current_depth = self.current_depth
    
    def create_behavior_tree(self):
        """Create the main behavior tree structure"""
        
        # Root: Selector with fallback to emergency stop
        root = py_trees.composites.Selector(
            name="Mission Root",
            memory=False
        )
        
        # Main mission sequence
        mission_sequence = py_trees.composites.Sequence(
            name="Main Mission",
            memory=True
        )
        
        # Mission setup
        mission_sequence.add_child(MissionSetup("Setup Mission", self))
        
        # Parallel: Check timeout while executing mission
        timeout_parallel = py_trees.composites.Parallel(
            name="Mission with Timeout",
            policy=py_trees.common.ParallelPolicy.SuccessOnOne(synchronise=False)
        )
        
        # Timeout checker
        timeout_parallel.add_child(
            CheckMissionTimeout("Check Timeout", self, self.mission_timeout))
        
        # Actual mission tasks
        tasks_sequence = py_trees.composites.Sequence(
            name="Task Execution",
            memory=True
        )
        
        # Task 1: Navigation
        navigation_sequence = py_trees.composites.Sequence(
            name="Task 1: Navigation",
            memory=True
        )
        
        navigation_sequence.add_child(SubmergeToDepth("Submerge", self))
        
        # Find and pass through gate
        gate_selector = py_trees.composites.Selector(
            name="Find Gate",
            memory=False
        )
        gate_selector.add_child(CheckGateDetected("Check Gate", self))
        gate_selector.add_child(SearchForGate("Search Gate", self))
        
        navigation_sequence.add_child(gate_selector)
        navigation_sequence.add_child(AlignToGate("Align to Gate", self))
        navigation_sequence.add_child(PassThroughGate("Pass Gate", self))
        
        tasks_sequence.add_child(navigation_sequence)
        
        # Task 4: Communication & Localization
        comm_sequence = py_trees.composites.Sequence(
            name="Task 4: Communication",
            memory=True
        )
        
        comm_sequence.add_child(WaitForFlareSequence("Get Sequence", self))
        
        # Process flares in sequence (simplified - bump all 3)
        for color in ['R', 'Y', 'B']:
            flare_sequence = py_trees.composites.Sequence(
                name=f"Flare {color}",
                memory=True
            )
            flare_sequence.add_child(SearchForFlare(f"Search {color}", self, color))
            flare_sequence.add_child(ApproachFlare(f"Bump {color}", self, color))
            comm_sequence.add_child(flare_sequence)
        
        tasks_sequence.add_child(comm_sequence)
        
        timeout_parallel.add_child(tasks_sequence)
        mission_sequence.add_child(timeout_parallel)
        
        root.add_child(mission_sequence)
        root.add_child(EmergencyStop("Emergency Stop", self))
        
        return root
    
    def tick_tree(self):
        """Tick the behavior tree"""
        self.root.tick_once()


def main(args=None):
    rclpy.init(args=args)
    
    node = AUVBehaviorTreeNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()