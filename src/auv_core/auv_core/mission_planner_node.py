# auv_core/mission_planner_node.py

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient # If using ROS Actions for movement
import py_trees
import py_trees_ros.trees
import py_trees.console as console
import sys
import time
import math
import numpy as np

# ROS Messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist, Point, Quaternion, Pose
from std_msgs.msg import Bool, String # Example for actuator commands or status

# TF for quaternion math
import tf_transformations

# --- Blackboard ---
# The blackboard is a central place to store and share data between behaviors.
blackboard = py_trees.blackboard.Client(name="MissionBlackboard")
blackboard.register_key(key="current_odom", access=py_trees.common.Access.WRITE)
blackboard.register_key(key="gate_detected", access=py_trees.common.Access.WRITE)
blackboard.register_key(key="gate_relative_pose", access=py_trees.common.Access.WRITE) # Example: Pose relative to AUV
blackboard.register_key(key="red_flare_detected", access=py_trees.common.Access.WRITE)
# Add keys for other objects, target poses, states etc.
blackboard.register_key(key="/control/target_pose", access=py_trees.common.Access.WRITE) # To store goal for GoToPose
blackboard.register_key(key="/control/target_velocity", access=py_trees.common.Access.WRITE) # To store goal for SetVelocity
blackboard.register_key(key="goal_reached", access=py_trees.common.Access.WRITE) # Flag for movement completion

# Initialize blackboard variables
blackboard.current_odom = None
blackboard.gate_detected = False
blackboard.gate_relative_pose = None
blackboard.red_flare_detected = False
blackboard.goal_reached = False


# --- Custom Behaviors ---
# These are the building blocks of your mission logic.

class ROSSubscriberCheck(py_trees.behaviour.Behaviour):
    """Checks if a specific ROS message has been received recently."""
    def __init__(self, name: str, topic_name: str, topic_type, blackboard_variable: str, timeout_sec: float = 2.0):
        super().__init__(name=name)
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.blackboard_variable = blackboard_variable
        self.timeout_sec = timeout_sec
        self.node = None # Will be set by the BehaviourTree node
        self.subscriber = None
        self.last_received_time = 0.0
        self.blackboard.register_key(key=blackboard_variable, access=py_trees.common.Access.WRITE)


    def setup(self, **kwargs):
        try:
            self.node = kwargs['node']
            self.subscriber = self.node.create_subscription(
                self.topic_type,
                self.topic_name,
                self.callback,
                10 # QoS Profile - adjust if needed
            )
            self.last_received_time = self.node.get_clock().now().nanoseconds / 1e9
            self.feedback_message = f"Waiting for {self.topic_name}"
        except KeyError as e:
            self.logger.error(f"Missing 'node' in setup arguments: {e}")

    def callback(self, msg):
        self.blackboard.set(self.blackboard_variable, msg)
        self.last_received_time = self.node.get_clock().now().nanoseconds / 1e9
        # Optional: Clear the variable if data becomes stale?

    def update(self) -> py_trees.common.Status:
        current_time = self.node.get_clock().now().nanoseconds / 1e9
        if current_time - self.last_received_time < self.timeout_sec:
             if self.blackboard.get(self.blackboard_variable) is not None:
                self.feedback_message = f"Received '{self.topic_name}' recently"
                return py_trees.common.Status.SUCCESS
             else:
                 # Received within timeout but blackboard is None (e.g., cleared)
                 self.feedback_message = f"Data for '{self.topic_name}' is None/Cleared"
                 return py_trees.common.Status.FAILURE
        else:
            self.feedback_message = f"No '{self.topic_name}' received in last {self.timeout_sec}s"
            self.blackboard.set(self.blackboard_variable, None) # Invalidate data on timeout
            return py_trees.common.Status.FAILURE

class GoToPose(py_trees.behaviour.Behaviour):
    """Sends a PoseStamped goal and waits for completion."""
    def __init__(self, name: str, pose: Pose, pos_tolerance=0.3, yaw_tolerance_deg=5.0):
        super().__init__(name=name)
        self.target_pose_world = pose # The target Pose in the world frame
        self.pos_tolerance_sq = pos_tolerance**2
        self.yaw_tolerance_rad = math.radians(yaw_tolerance_deg)
        self.node = None
        self.publisher = None
        self.goal_sent = False

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.publisher = self.node.create_publisher(PoseStamped, '/control/target_pose', 10)

    def initialise(self):
        self.goal_sent = False
        self.blackboard.goal_reached = False # Reset flag on entry
        self.feedback_message = "Initialising..."

    def update(self) -> py_trees.common.Status:
        current_odom: Odometry = self.blackboard.current_odom
        if current_odom is None:
            self.feedback_message = "Waiting for odometry"
            return py_trees.common.Status.RUNNING # Can't do anything without knowing current pose

        if not self.goal_sent:
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.node.get_clock().now().to_msg()
            goal_msg.header.frame_id = current_odom.header.frame_id # Assume target is in odom frame
            goal_msg.pose = self.target_pose_world
            self.publisher.publish(goal_msg)
            self.blackboard.set("/control/target_pose", goal_msg) # Store the goal
            self.blackboard.set("/control/target_velocity", None) # Clear velocity goal
            self.goal_sent = True
            self.feedback_message = "Goal sent, moving..."
            return py_trees.common.Status.RUNNING

        # Check if goal is reached
        current_pos = current_odom.pose.pose.position
        goal_pos = self.target_pose_world.position
        dist_sq = (current_pos.x - goal_pos.x)**2 + \
                  (current_pos.y - goal_pos.y)**2 + \
                  (current_pos.z - goal_pos.z)**2

        # Check Yaw
        current_q = current_odom.pose.pose.orientation
        goal_q = self.target_pose_world.orientation
        _, _, current_yaw = tf_transformations.euler_from_quaternion([current_q.x, current_q.y, current_q.z, current_q.w])
        _, _, goal_yaw = tf_transformations.euler_from_quaternion([goal_q.x, goal_q.y, goal_q.z, goal_q.w])
        yaw_diff = current_yaw - goal_yaw
        while yaw_diff > math.pi: yaw_diff -= 2 * math.pi
        while yaw_diff <= -math.pi: yaw_diff += 2 * math.pi

        if dist_sq < self.pos_tolerance_sq and abs(yaw_diff) < self.yaw_tolerance_rad:
            self.feedback_message = "Goal Reached!"
            self.blackboard.goal_reached = True # Set flag
            return py_trees.common.Status.SUCCESS
        else:
            # Provide distance and yaw error in feedback message for debugging
            dist = math.sqrt(dist_sq)
            self.feedback_message = f"Moving... Dist: {dist:.2f}m, Yaw Err: {math.degrees(yaw_diff):.1f}deg"
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status: py_trees.common.Status):
        # Optionally send a zero velocity command on termination? Depends on control node behavior
        self.feedback_message = f"Terminated with status {new_status}"


class SetVelocity(py_trees.behaviour.Behaviour):
    """Sends a constant Twist command."""
    def __init__(self, name: str, linear: tuple = (0.0, 0.0, 0.0), angular: tuple = (0.0, 0.0, 0.0)):
        super().__init__(name=name)
        self.linear = linear
        self.angular = angular
        self.node = None
        self.publisher = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.publisher = self.node.create_publisher(Twist, '/control/target_velocity', 10)

    def update(self) -> py_trees.common.Status:
        vel_msg = Twist()
        vel_msg.linear.x, vel_msg.linear.y, vel_msg.linear.z = self.linear
        vel_msg.angular.x, vel_msg.angular.y, vel_msg.angular.z = self.angular
        self.publisher.publish(vel_msg)
        self.blackboard.set("/control/target_velocity", vel_msg) # Store the goal
        self.blackboard.set("/control/target_pose", None) # Clear pose goal
        self.feedback_message = f"Setting Vel: lin={self.linear}, ang={self.angular}"
        return py_trees.common.Status.SUCCESS # Usually succeeds immediately unless publisher fails

class CheckDepth(py_trees.behaviour.Behaviour):
    """Checks if current depth is within a range."""
    def __init__(self, name: str, min_depth: float, max_depth: float):
        super().__init__(name=name)
        self.min_depth = min_depth # Assuming depth is negative Z
        self.max_depth = max_depth

    def update(self) -> py_trees.common.Status:
        current_odom: Odometry = self.blackboard.current_odom
        if current_odom is None:
            self.feedback_message = "Waiting for odometry"
            return py_trees.common.Status.FAILURE # Or RUNNING? Failure seems safer

        current_depth = current_odom.pose.pose.position.z
        if self.min_depth <= current_depth <= self.max_depth:
            self.feedback_message = f"Depth {current_depth:.2f} within [{self.min_depth:.2f}, {self.max_depth:.2f}]"
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = f"Depth {current_depth:.2f} outside [{self.min_depth:.2f}, {self.max_depth:.2f}]"
            return py_trees.common.Status.FAILURE

class CheckBoolBlackboardVariable(py_trees.behaviour.Behaviour):
    """Checks if a boolean variable on the blackboard is True."""
    def __init__(self, name: str, variable_name: str):
        super(CheckBoolBlackboardVariable, self).__init__(name=name)
        self.variable_name = variable_name
        # Ensure key is registered by the subscriber behavior or manually
        blackboard.register_key(key=variable_name, access=py_trees.common.Access.READ)


    def update(self):
        value = blackboard.get(self.variable_name)
        if value is True:
            self.feedback_message = f"'{self.variable_name}' is True"
            return py_trees.common.Status.SUCCESS
        else:
            self.feedback_message = f"'{self.variable_name}' is False or None"
            return py_trees.common.Status.FAILURE

class SimpleActuatorCommand(py_trees.behaviour.Behaviour):
    """Publishes a simple command (e.g., True/False or String) to an actuator topic."""
    def __init__(self, name: str, topic_name: str, topic_type, message_value):
        super().__init__(name=name)
        self.topic_name = topic_name
        self.topic_type = topic_type
        self.message_value = message_value
        self.node = None
        self.publisher = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.publisher = self.node.create_publisher(self.topic_type, self.topic_name, 10)

    def update(self) -> py_trees.common.Status:
        msg = self.topic_type()
        # Handle different message types - simple example for Bool or String
        if isinstance(msg, Bool):
            msg.data = bool(self.message_value)
        elif isinstance(msg, String):
            msg.data = str(self.message_value)
        else:
             # Basic data types might have 'data' attribute
             try:
                 setattr(msg, 'data', self.message_value)
             except AttributeError:
                 self.logger.error(f"Cannot set 'data' for message type {self.topic_type}")
                 self.feedback_message = "Msg type error"
                 return py_trees.common.Status.FAILURE

        self.publisher.publish(msg)
        self.feedback_message = f"Sent '{self.message_value}' to '{self.topic_name}'"
        return py_trees.common.Status.SUCCESS # Action is usually instantaneous


# --- Helper Function to Create the Behavior Tree ---
def create_root(node: Node) -> py_trees.behaviour.Behaviour:
    """Creates the behavior tree for the SAUVC mission."""

    # --- Root Sequence ---
    root = py_trees.composites.Sequence(name="Mission Root", memory=True)

    # --- Setup Behaviors (Run Once) ---
    setup_tasks = py_trees.composites.Sequence(name="Setup", memory=True)
    check_odom = ROSSubscriberCheck(name="Check Odom",
                                    topic_name="/odometry/filtered",
                                    topic_type=Odometry,
                                    blackboard_variable="current_odom")
    # Add checks for other essential sensors/systems if needed

    setup_tasks.add_children([check_odom])

    # --- Mission Tasks ---
    mission_tasks = py_trees.composites.Sequence(name="Mission Tasks", memory=True)

    # --- Task 1: Submerge ---
    submerge = GoToPose(name="Submerge",
                       pose=Pose(position=Point(x=0.0, y=0.0, z=-1.5), # Target Z relative to odom origin (surface=0?)
                                 orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)), # Maintain initial yaw
                       pos_tolerance=0.2, yaw_tolerance_deg=180.0) # Loose yaw tolerance here

    # --- Task 2: Navigate Gate ---
    navigate_gate = py_trees.composites.Sequence(name="Navigate Gate", memory=True)
    gate_wp_pose = Pose(position=Point(x=10.0, y=0.0, z=-1.5), # Waypoint BEFORE gate
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
    go_to_gate_wp = GoToPose(name="Go To Gate WP", pose=gate_wp_pose, pos_tolerance=0.5)

    # --- Alignment Sub-tree (Selector: Try visual servo, else maybe just wait/proceed) ---
    align_gate_selector = py_trees.composites.Selector(name="Align Gate Logic", memory=False) # Non-memory: re-evaluate each tick
    visual_servo_gate = py_trees.composites.Sequence(name="Visual Servo Gate", memory=False)
    check_gate_visible = CheckBoolBlackboardVariable(name="Is Gate Visible?", variable_name="gate_detected")
    # TODO: Add actual visual servoing behavior here
    # This behavior would subscribe to gate_relative_pose, calculate Twist commands (vy, yaw_rate)
    # and publish them. It would return SUCCESS when aligned, FAILURE if lost, RUNNING otherwise.
    # For now, placeholder:
    servo_action_placeholder = py_trees.behaviours.Running(name="Visual Servoing (Placeholder)")

    visual_servo_gate.add_children([check_gate_visible, servo_action_placeholder])

    # Fallback if visual servo fails or not implemented
    wait_for_alignment = py_trees.behaviours.Timer(name="Wait Briefly", duration=2.0)

    align_gate_selector.add_children([visual_servo_gate, wait_for_alignment])

    # --- Pass Through Gate ---
    gate_pass_pose = Pose(position=Point(x=14.0, y=0.0, z=-1.5), # Waypoint AFTER gate
                          orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
    pass_gate = GoToPose(name="Pass Gate WP", pose=gate_pass_pose)

    navigate_gate.add_children([go_to_gate_wp, align_gate_selector, pass_gate])

    # --- Task 3 onwards (Selector: Choose next task based on strategy/detection) ---
    # Example: Selector between Target Acquisition and Flare Search
    subsequent_tasks = py_trees.composites.Selector(name="Choose Next Task", memory=True) # Memory: If one succeeds, don't try others

    # --- Target Acquisition Sub-tree ---
    target_acquisition = py_trees.composites.Sequence(name="Target Acquisition", memory=True)
    target_zone_wp = Pose(position=Point(x=30.0, y=-4.0, z=-1.8), # Go deeper near target zone
                           orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0))
    go_to_target_zone = GoToPose(name="Go To Target Zone", pose=target_zone_wp, pos_tolerance=1.0)
    # TODO: Add search pattern behavior
    # TODO: Add drum detection check
    # TODO: Add visual servo over correct drum behavior
    drop_ball = SimpleActuatorCommand(name="Drop Ball", topic_name="/actuators/dropper", topic_type=Bool, message_value=True) # Example
    wait_after_drop = py_trees.behaviours.Timer(name="Wait After Drop", duration=1.0)

    target_acquisition.add_children([go_to_target_zone, drop_ball, wait_after_drop]) # Add search/align behaviors

    # --- Flare Search Sub-tree (Example for Red Flare) ---
    flare_search = py_trees.composites.Sequence(name="Flare Search", memory=True)
    # TODO: Add search pattern
    check_red_flare = CheckBoolBlackboardVariable(name="Is Red Flare Visible?", variable_name="red_flare_detected")
    # TODO: Add align/bump behavior
    bump_red_flare = SetVelocity(name="Bump Red Flare Gently", linear=(0.2, 0.0, 0.0)) # Example: slow forward
    wait_after_bump = py_trees.behaviours.Timer(name="Wait After Bump", duration=1.5)
    stop_after_bump = SetVelocity(name="Stop After Bump", linear=(0.0, 0.0, 0.0))

    flare_search.add_children([check_red_flare, bump_red_flare, wait_after_bump, stop_after_bump]) # Add search/align

    # Add other task sub-trees here (Target Reacquisition, Comm & Loc sequence)

    subsequent_tasks.add_children([target_acquisition, flare_search]) # Add other task roots

    # --- Surfacing ---
    surface = GoToPose(name="Surface",
                       pose=Pose(position=Point(x=0.0, y=0.0, z=-0.1), # Go near surface (relative to current XY?)
                                 orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
                       pos_tolerance=0.3)

    # --- Build the main mission sequence ---
    mission_tasks.add_children([
        submerge,
        navigate_gate,
        subsequent_tasks, # Choose and execute one of the next tasks
        # surface # Optionally add surfacing at the end
    ])

    # --- Assemble Root ---
    root.add_children([setup_tasks, mission_tasks])

    return root


# --- ROS 2 Node Class ---
class MissionPlannerBTNode(Node):
    def __init__(self):
        super().__init__('mission_planner_bt_node')
        self.tree = None

        # --- Parameters ---
        self.declare_parameter('bt_tick_rate_hz', 10.0)

        # --- Subscribers needed to update Blackboard ---
        # Note: ROSSubscriberCheck behavior handles its own subscription,
        # but we might still need direct subs for things used by many behaviors.
        # Odometry is crucial, handled by ROSSubscriberCheck above.

        # Example: Direct subscription for gate detection if not using ROSSubscriberCheck behavior
        # self.gate_sub = self.create_subscription(
        #      Bool, '/perception/gate_detected', self.gate_detected_callback, 10)

        # --- Behavior Tree Setup ---
        self.tree = py_trees_ros.trees.BehaviourTree(
            root=create_root(self),
            node=self # Pass the ROS node to the BT and its behaviors
        )
        try:
            self.tree.setup(timeout=15.0) # Setup publishers/subscribers in behaviors
        except py_trees.exceptions.TimedOutError as e:
            console.logerror(console.red + f"failed to setup tree, aborting [{e}]" + console.reset)
            self.tree = None
            rclpy.shutdown()
            sys.exit(1)

        # --- Start Ticking ---
        tick_rate = self.get_parameter('bt_tick_rate_hz').value
        self.timer = self.create_timer(1.0 / tick_rate, self.tree.tick)

        self.get_logger().info('Behavior Tree Mission Planner started.')

    # Example direct callback to update blackboard
    # def gate_detected_callback(self, msg: Bool):
    #      blackboard.gate_detected = msg.data

    def destroy_node(self):
         # Cleanup the Behavior Tree
         if self.tree is not None:
             self.get_logger().info("Shutting down Behavior Tree...")
             self.tree.shutdown()
         super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlannerBTNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
         node.get_logger().error(f"Unhandled exception in spin: {e}", exc_info=True)
    finally:
        # Explicitly destroy node to trigger BT shutdown
         if rclpy.ok() and node.tree is not None:
             node.destroy_node()
             rclpy.shutdown()
         elif rclpy.ok(): # If tree setup failed
              rclpy.shutdown()


if __name__ == '__main__':
    main()