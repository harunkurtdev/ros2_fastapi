import rclpy
from rclpy.node import Node
import threading
import uvicorn
from rclpy.executors import MultiThreadedExecutor
from typing import Optional
from pydantic import BaseModel
from std_msgs.msg import String

from fastapi import FastAPI, Path, HTTPException, status, Request, Depends
from fastapi.middleware.cors import CORSMiddleware

try:
    from server_router import (
        api_router, 
        MinimalPublisher, 
        MinimalSubscriber,
        set_publisher_instance,
        set_subscriber_instance
    )
except ImportError:
    # If running as a package, use relative import
    from .server_router import (
        api_router, 
        MinimalPublisher, 
        MinimalSubscriber,
        set_publisher_instance,
        set_subscriber_instance
    )


class Response(BaseModel):
    msg: str


# FastAPI app oluştur
fastapiapp = FastAPI(
    title="ROS2 FastAPI Server",
    description="A FastAPI server that interfaces with ROS2 nodes",
    version="1.0.0"
)

# Router'ı include et
fastapiapp.include_router(api_router, prefix="/api")

# CORS middleware ekle
fastapiapp.add_middleware(
    CORSMiddleware,
    expose_headers=["*"],
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# Ana endpoint'ler (doğrudan FastAPI app'e eklenen)
@fastapiapp.get("/")
async def root():
    return {"message": "ROS2 FastAPI Server is running", "docs": "/docs", "api": "/api"}


@fastapiapp.get("/health")
async def health_check():
    return {"status": "healthy", "service": "ros2-fastapi-server"}


def main(args=None):
    rclpy.init()
    executor = MultiThreadedExecutor()
    
    # ROS2 node'larını oluştur
    minimal_publisher = MinimalPublisher()
    minimal_subscriber = MinimalSubscriber()
    
    # Global instance'ları set et (router endpoint'leri için)
    set_publisher_instance(minimal_publisher)
    set_subscriber_instance(minimal_subscriber)
    
    # Node'ları executor'a ekle
    executor.add_node(minimal_publisher)
    executor.add_node(minimal_subscriber)

    # ROS 2 executor ayrı bir thread'de çalışmalı
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()
    
    print("🚀 Starting ROS2 Multi-Node FastAPI Server...")
    minimal_publisher.get_logger().info("Starting FastAPI server on port 5002...")
    
    try:
        uvicorn.run(
            fastapiapp, 
            host="0.0.0.0",
            port=5002, 
            log_level="info"
        )
    except KeyboardInterrupt:
        print("\n⏹️  Shutting down server...")
    finally:
        rclpy.shutdown()
        ros_thread.join()
        print("✅ Server shutdown complete")


if __name__ == "__main__":
    main()