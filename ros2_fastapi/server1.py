import rclpy
from rclpy.node import Node
import sys
import threading
import uvicorn
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from typing import Optional
from pydantic import BaseModel

from std_msgs.msg import String

from fastapi import FastAPI, Path, HTTPException, status, Request, Depends
from fastapi.middleware.cors import CORSMiddleware


from fastapi.responses import RedirectResponse

from .api import (
    api_router,
)


class Response(BaseModel):
    msg: str


fastapiapp = FastAPI()
fastapiapp.include_router(api_router, prefix="/api")


# Ui için
# "http://localhost:5000"
fastapiapp.add_middleware(
    CORSMiddleware,
    expose_headers=["*"],
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(String, "aa", 10)
        self.subscriber_ = self.create_subscription(
            String, "aa", self.listener_callback, 10
        )
        self.i = 0
        self.latest_message = ""  # To store the latest received message

        @fastapiapp.get("/publish", response_model=Response)
        async def publish():
            msg = String()
            msg.data = "Hello World: %d" % self.i
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            self.i += 1

            response = {"msg": "Message published"}

            return response

        @fastapiapp.get("/latest_message")
        async def get_latest_message():
            return {
                "msg": self.latest_message,
                "timestamp": self.get_clock().now().to_msg().sec,
                "i": self.i,
            }

    def listener_callback(self, msg):
        self.latest_message = msg.data
        self.get_logger().info('Received message: "%s"' % msg.data)


def main(args=None):
    rclpy.init()
    minimal_publisher = MinimalPublisher()
    executor = MultiThreadedExecutor()
    executor.add_node(minimal_publisher)

    # ROS 2 executor ayrı bir thread'de çalışmalı
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    minimal_publisher.get_logger().info("Starting FastAPI server on port 5002...")

    try:
        uvicorn.run(fastapiapp, port=5002, log_level="warning")
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.get_logger().info("Shutting down...")
        rclpy.shutdown()
        ros_thread.join()
