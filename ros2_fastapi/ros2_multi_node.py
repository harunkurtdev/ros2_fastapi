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


# Service Node
class ServiceNode(Node):
    def __init__(self, node_name: str = "service_node"):
        super().__init__(node_name)
        self.callback_group = ReentrantCallbackGroup()

        # Service client (example)
        # self.client = self.create_client(AddTwoInts, 'add_two_ints', callback_group=self.callback_group)

        self.service_calls = []
        self.service_call_count = 0

    def simulate_service_call(self, a: int, b: int):
        """Simulate a service call"""
        result = a + b
        call_info = {
            "call_id": self.service_call_count,
            "input": {"a": a, "b": b},
            "result": result,
            "timestamp": datetime.now().isoformat(),
        }

        self.service_calls.append(call_info)
        self.service_call_count += 1

        # Keep only last 50 calls
        if len(self.service_calls) > 50:
            self.service_calls = self.service_calls[-50:]

        self.get_logger().info(f"Service call: {a} + {b} = {result}")
        return call_info


# Parameter Node
class ParameterNode(Node):
    def __init__(self, node_name: str = "parameter_node"):
        super().__init__(node_name)
        self.callback_group = ReentrantCallbackGroup()

        # Declare parameters
        self.declare_parameter("robot_name", "default_robot")
        self.declare_parameter("max_speed", 1.0)
        self.declare_parameter("debug_mode", False)

    def get_parameters_info(self):
        """Get current parameter values"""
        return {
            "robot_name": self.get_parameter("robot_name").value,
            "max_speed": self.get_parameter("max_speed").value,
            "debug_mode": self.get_parameter("debug_mode").value,
        }

    def update_parameter(self, param_name: str, value: Any):
        """Update a parameter value"""
        try:
            # Get current parameter to check type
            current_param = self.get_parameter(param_name)

            # Convert value to appropriate type
            if current_param.type_ == rclpy.Parameter.Type.STRING:
                new_value = str(value)
            elif current_param.type_ == rclpy.Parameter.Type.DOUBLE:
                new_value = float(value)
            elif current_param.type_ == rclpy.Parameter.Type.BOOL:
                new_value = bool(value)
            elif current_param.type_ == rclpy.Parameter.Type.INTEGER:
                new_value = int(value)
            else:
                new_value = value

            # Set the parameter
            self.set_parameters([rclpy.Parameter(param_name, value=new_value)])
            self.get_logger().info(f"Updated parameter {param_name} to {new_value}")
            return True
        except Exception as e:
            self.get_logger().error(
                f"Failed to update parameter {param_name}: {str(e)}"
            )
            return False


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


# Global node manager
node_manager = NodeManager()


def get_node_manager() -> NodeManager:
    """Dependency to get the node manager"""
    if not node_manager.is_running:
        raise HTTPException(status_code=500, detail="Node manager not initialized")
    return node_manager


def initialize_all_nodes():
    """Initialize all nodes and start executor"""
    try:
        node_manager.initialize_ros()
        node_manager.create_nodes()
        node_manager.start_executor()
        return node_manager
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Failed to initialize nodes: {str(e)}"
        )


# Create API router
api_router = APIRouter(tags=["ros2_multi_node"])


@api_router.get("/", response_model=dict)
async def root():
    return {"message": "ROS2 Multi-Node API", "nodes": list(node_manager.nodes.keys())}


@api_router.get("/nodes/status", response_model=dict)
async def get_nodes_status(manager: NodeManager = Depends(get_node_manager)):
    """Get status of all nodes"""
    return manager.get_all_nodes_status()


# Publisher Node Endpoints
@api_router.post("/publisher/string", response_model=dict)
async def publish_string(
    message: str = "Hello World", manager: NodeManager = Depends(get_node_manager)
):
    """Publish a string message"""
    try:
        publisher_node = manager.get_node("publisher")
        published_msg = publisher_node.publish_string_message(message)
        return {
            "status": "success",
            "published_message": published_msg,
            "publish_count": publisher_node.publish_count,
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to publish: {str(e)}")


@api_router.post("/publisher/cmd_vel", response_model=dict)
async def publish_cmd_vel(
    linear_x: float = 0.0,
    angular_z: float = 0.0,
    manager: NodeManager = Depends(get_node_manager),
):
    """Publish velocity command"""
    try:
        publisher_node = manager.get_node("publisher")
        result = publisher_node.publish_cmd_vel(linear_x, angular_z)
        return {"status": "success", "command": result}
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Failed to publish cmd_vel: {str(e)}"
        )


# Subscriber Node Endpoints
@api_router.get("/subscriber/latest", response_model=dict)
async def get_latest_messages(
    count: int = 10, manager: NodeManager = Depends(get_node_manager)
):
    """Get latest received messages"""
    try:
        subscriber_node = manager.get_node("subscriber")
        messages = subscriber_node.get_latest_messages(count)
        return {
            "latest_messages": messages,
            "total_count": subscriber_node.message_count,
            "latest_string": subscriber_node.latest_string_message,
            "latest_cmd_vel": subscriber_node.latest_cmd_vel,
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get messages: {str(e)}")


# Service Node Endpoints
@api_router.post("/service/add", response_model=dict)
async def call_add_service(
    a: int, b: int, manager: NodeManager = Depends(get_node_manager)
):
    """Simulate service call"""
    try:
        service_node = manager.get_node("service")
        result = service_node.simulate_service_call(a, b)
        return {"status": "success", "service_call": result}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Service call failed: {str(e)}")


@api_router.get("/service/history", response_model=dict)
async def get_service_history(manager: NodeManager = Depends(get_node_manager)):
    """Get service call history"""
    try:
        service_node = manager.get_node("service")
        return {
            "service_calls": service_node.service_calls,
            "total_calls": service_node.service_call_count,
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get history: {str(e)}")


# Parameter Node Endpoints
@api_router.get("/parameters", response_model=dict)
async def get_parameters(manager: NodeManager = Depends(get_node_manager)):
    """Get all parameter values"""
    try:
        param_node = manager.get_node("parameter")
        return {"parameters": param_node.get_parameters_info()}
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Failed to get parameters: {str(e)}"
        )


@api_router.post("/parameters/{param_name}", response_model=dict)
async def update_parameter(
    param_name: str, value: str, manager: NodeManager = Depends(get_node_manager)
):
    """Update a parameter value"""
    try:
        param_node = manager.get_node("parameter")
        success = param_node.update_parameter(param_name, value)
        if success:
            return {
                "status": "success",
                "parameter": param_name,
                "new_value": value,
                "all_parameters": param_node.get_parameters_info(),
            }
        else:
            raise HTTPException(
                status_code=400, detail=f"Failed to update parameter {param_name}"
            )
    except Exception as e:
        raise HTTPException(
            status_code=500, detail=f"Parameter update failed: {str(e)}"
        )


# System Control Endpoints
@api_router.post("/system/restart", response_model=dict)
async def restart_system():
    """Restart the entire node system"""
    try:
        # Stop current system
        node_manager.stop_executor()
        time.sleep(1)  # Give time for cleanup

        # Restart
        initialize_all_nodes()
        return {"status": "success", "message": "System restarted successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Restart failed: {str(e)}")


@api_router.post("/system/stop", response_model=dict)
async def stop_system():
    """Stop the entire node system"""
    try:
        node_manager.stop_executor()
        return {"status": "success", "message": "System stopped successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Stop failed: {str(e)}")


# Initialize system on startup
@api_router.on_event("startup")
async def startup_event():
    """Initialize nodes on startup"""
    initialize_all_nodes()


@api_router.on_event("shutdown")
async def shutdown_event():
    """Cleanup on shutdown"""
    node_manager.stop_executor()
