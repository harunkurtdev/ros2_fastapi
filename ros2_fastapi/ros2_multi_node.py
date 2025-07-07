from typing import List, Dict, Optional, Any
from datetime import datetime
from fastapi import APIRouter, HTTPException, Depends
import time
from dataclasses import dataclass

try:
    from .node_manager import NodeManager
    from .publisher import PublisherNode
    from .subscriber import SubscriberNode
    from .service import ServiceNode
    from .parameter import ParameterNode
except ImportError:
    from node_manager import NodeManager
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
