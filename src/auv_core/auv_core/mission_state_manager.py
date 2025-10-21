#!/usr/bin/env python3
"""
Mission State Manager - Persists and recovers mission state
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import os
from pathlib import Path

class MissionStateManager(Node):
    def __init__(self):
        super().__init__('mission_state_manager')
        
        # State file location
        self.state_dir = Path.home() / '.ros' / 'auv_mission_state'
        self.state_dir.mkdir(parents=True, exist_ok=True)
        self.state_file = self.state_dir / 'mission_state.json'
        
        # Current state
        self.state = {
            'tasks_completed': [],
            'navigation_complete': False,
            'flares_bumped': [],
            'current_task': 'navigation',
            'last_known_position': None,
            'attempt_count': 0
        }
        
        # Load previous state
        self.load_state()
        
        # Subscriptions
        self.state_update_sub = self.create_subscription(
            String, '/mission/state_update', self.state_update_callback, 10)
        
        # Publishers
        self.state_pub = self.create_publisher(String, '/mission/current_state', 10)
        
        # Periodic save
        self.timer = self.create_timer(5.0, self.save_state)
        
        self.get_logger().info(f'Mission State Manager initialized. State: {self.state}')
    
    def load_state(self):
        if self.state_file.exists():
            with open(self.state_file, 'r') as f:
                loaded_state = json.load(f)
                self.state.update(loaded_state)
                self.get_logger().info(f'Loaded previous mission state: {self.state}')
    
    def save_state(self):
        with open(self.state_file, 'w') as f:
            json.dump(self.state, f, indent=2)
        
        # Publish current state
        state_msg = String()
        state_msg.data = json.dumps(self.state)
        self.state_pub.publish(state_msg)
    
    def state_update_callback(self, msg: String):
        update = json.loads(msg.data)
        self.state.update(update)
        self.save_state()
        self.get_logger().info(f'State updated: {update}')
    
    def reset_state(self):
        self.state = {
            'tasks_completed': [],
            'navigation_complete': False,
            'flares_bumped': [],
            'current_task': 'navigation',
            'last_known_position': None,
            'attempt_count': 0
        }
        self.save_state()


def main(args=None):
    rclpy.init(args=args)
    node = MissionStateManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save_state()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()