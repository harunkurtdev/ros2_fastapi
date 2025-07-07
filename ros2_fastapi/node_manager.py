from typing import List, Dict, Optional, Any
from datetime import datetime
from fastapi import APIRouter, HTTPException, Depends
from std_msgs.msg import String, Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, Imu
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
import rclpy
import time
from dataclasses import dataclass

try:
    from .publisher import PublisherNode
    from .subscriber import SubscriberNode
    from .service import ServiceNode
    from .parameter import ParameterNode
except ImportError:
    from publisher import PublisherNode
    from subscriber import SubscriberNode
    from service import ServiceNode
    from parameter import ParameterNode


# Models for responses
@dataclass
class Response:
    msg: str
    timestamp: str = None

    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = datetime.now().isoformat()


# Node Manager Class
class NodeManager:
    def __init__(self):
        self.executor: Optional[MultiThreadedExecutor] = None
        self.nodes: Dict[str, Node] = {}
        self.executor_thread: Optional[threading.Thread] = None
        self.is_running = False

    def initialize_ros(self):
        """Initialize ROS2"""
        if not rclpy.ok():
            rclpy.init()

    def create_nodes(self):
        """Create all nodes"""
        self.nodes = {
            "publisher": PublisherNode(),
            "subscriber": SubscriberNode(),
            "service": ServiceNode(),
            "parameter": ParameterNode(),
        }

    def start_executor(self):
        """Start the multi-threaded executor"""
        if self.is_running:
            return

        self.executor = MultiThreadedExecutor(num_threads=4)

        # Add all nodes to executor
        for node in self.nodes.values():
            self.executor.add_node(node)

        # Start spinning in a separate thread
        def spin_executor():
            try:
                self.executor.spin()
            except Exception as e:
                print(f"Executor error: {e}")

        self.executor_thread = threading.Thread(target=spin_executor, daemon=True)
        self.executor_thread.start()
        self.is_running = True

        print("ROS2 MultiThreadedExecutor started with all nodes")

    def stop_executor(self):
        """Stop the executor and clean up"""
        if self.executor:
            self.executor.shutdown()
            self.is_running = False

        for node in self.nodes.values():
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

    def get_node(self, node_name: str) -> Node:
        """Get a specific node"""
        if node_name not in self.nodes:
            raise HTTPException(status_code=404, detail=f"Node '{node_name}' not found")
        return self.nodes[node_name]

    def get_all_nodes_status(self):
        """Get status of all nodes"""
        status = {}
        for name, node in self.nodes.items():
            status[name] = {
                "node_name": node.get_name(),
                "namespace": node.get_namespace(),
                "is_running": self.is_running,
            }
        return status
