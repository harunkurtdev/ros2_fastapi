import time
from typing import Optional

from fastapi import APIRouter

from rclpy.node import Node

from std_msgs.msg import String


# Create API router
api_router = APIRouter(tags=["server_router"])

# Global instances
minimal_publisher_instance = None
minimal_subscriber_instance = None


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(String, "topic", 10)
        self.i = 0

    def publish_message(self):
        msg = String()
        msg.data = "Hello World: %d" % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
        return {"msg": "Message published", "data": msg.data, "count": self.i}


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__("minimal_subscriber")
        self.subscription = self.create_subscription(
            String, "topic", self.listener_callback, 10
        )
        self.subscription  # prevent unused variable warning
        self.latest_message = ""  # To store the latest received message
        self.i = 0

    def listener_callback(self, msg):
        self.latest_message = msg.data
        self.get_logger().info('Received message: "%s"' % msg.data)
        self.i += 1

    def get_latest_message(self):
        return {
            "msg": self.latest_message,
            "timestamp": self.get_clock().now().to_msg().sec,
            "count": self.i,
        }


# API Endpoints - Class'ların dışında tanımlanmalı
@api_router.get("/publish", response_model=dict)
async def publish():
    """Publish a message to the ROS topic"""
    if minimal_publisher_instance is None:
        return {"error": "Publisher not initialized"}
    
    return minimal_publisher_instance.publish_message()


@api_router.get("/latest_message")
async def get_latest_message():
    """Get the latest received message"""
    if minimal_subscriber_instance is None:
        return {"error": "Subscriber not initialized"}
    
    return minimal_subscriber_instance.get_latest_message()


@api_router.get("/status")
async def get_status():
    """Get the status of publisher and subscriber"""
    return {
        "publisher_initialized": minimal_publisher_instance is not None,
        "subscriber_initialized": minimal_subscriber_instance is not None,
        "status": "running"
    }


# Utility functions to set instances
def set_publisher_instance(instance: MinimalPublisher):
    global minimal_publisher_instance
    minimal_publisher_instance = instance


def set_subscriber_instance(instance: MinimalSubscriber):
    global minimal_subscriber_instance
    minimal_subscriber_instance = instance