from typing import List, Dict, Optional, Any
from datetime import datetime
from fastapi import Depends
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from dataclasses import dataclass


# Models for responses
@dataclass
class Response:
    msg: str
    timestamp: str = None

    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now().isoformat()


# Publisher Node
class PublisherNode(Node):
    def __init__(self, node_name: str = "publisher_node"):
        super().__init__(node_name)
        self.callback_group = ReentrantCallbackGroup()

        # Publishers
        self.string_publisher = self.create_publisher(
            String, "chatter", 10, callback_group=self.callback_group
        )
        self.cmd_vel_publisher = self.create_publisher(
            Twist, "cmd_vel", 10, callback_group=self.callback_group
        )

        self.publish_count = 0
        self.latest_published_message = ""

        # Timer for periodic publishing
        self.timer = self.create_timer(
            2.0, self.timer_callback, callback_group=self.callback_group
        )

    def publish_string_message(self, message: str = None):
        """Publish a string message"""
        if message is None:
            message = f"Hello World: {self.publish_count}"

        msg = String()
        msg.data = message
        self.string_publisher.publish(msg)
        self.latest_published_message = message
        self.publish_count += 1

        self.get_logger().info(f'Published string: "{message}"')
        return message

    def publish_cmd_vel(self, linear_x: float = 0.0, angular_z: float = 0.0):
        """Publish velocity command"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z

        self.cmd_vel_publisher.publish(msg)
        self.get_logger().info(
            f"Published cmd_vel: linear={linear_x}, angular={angular_z}"
        )
        return f"cmd_vel: linear_x={linear_x}, angular_z={angular_z}"

    def timer_callback(self):
        """Timer callback for periodic publishing"""
        self.publish_string_message(f"Timer message: {self.publish_count}")
