from datetime import datetime
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from dataclasses import dataclass


# Subscriber Node
class SubscriberNode(Node):
    def __init__(self, node_name: str = "subscriber_node"):
        super().__init__(node_name)
        self.callback_group = ReentrantCallbackGroup()

        # Subscribers
        self.string_subscriber = self.create_subscription(
            String,
            "chatter",
            self.string_callback,
            10,
            callback_group=self.callback_group,
        )
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            "cmd_vel",
            self.cmd_vel_callback,
            10,
            callback_group=self.callback_group,
        )

        self.received_messages = []
        self.latest_string_message = ""
        self.latest_cmd_vel = None
        self.message_count = 0

    def string_callback(self, msg):
        """Callback for string messages"""
        self.latest_string_message = msg.data
        self.received_messages.append(
            {
                "type": "string",
                "data": msg.data,
                "timestamp": datetime.now().isoformat(),
            }
        )
        self.message_count += 1

        # Keep only last 100 messages
        if len(self.received_messages) > 100:
            self.received_messages = self.received_messages[-100:]

        self.get_logger().info(f'Received string: "{msg.data}"')

    def cmd_vel_callback(self, msg):
        """Callback for cmd_vel messages"""
        self.latest_cmd_vel = {
            "linear_x": msg.linear.x,
            "angular_z": msg.angular.z,
            "timestamp": datetime.now().isoformat(),
        }
        self.received_messages.append(
            {
                "type": "cmd_vel",
                "data": self.latest_cmd_vel,
                "timestamp": datetime.now().isoformat(),
            }
        )
        self.message_count += 1

        # Keep only last 100 messages
        if len(self.received_messages) > 100:
            self.received_messages = self.received_messages[-100:]

        self.get_logger().info(
            f"Received cmd_vel: linear={msg.linear.x}, angular={msg.angular.z}"
        )

    def get_latest_messages(self, count: int = 10):
        """Get latest received messages"""
        return self.received_messages[-count:]
