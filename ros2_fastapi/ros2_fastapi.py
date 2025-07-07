# main.py
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
import uvicorn
try:
    from ros2_multi_node import api_router, initialize_all_nodes, node_manager
except ImportError:
    from .ros2_multi_node import api_router, initialize_all_nodes, node_manager
    
# Create FastAPI app
app = FastAPI(
    title="ROS2 Multi-Node API",
    description="FastAPI integration with multiple ROS2 nodes using MultiThreadedExecutor",
    version="1.0.0",
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include the ROS2 router
app.include_router(api_router, prefix="/api/v1")


@app.on_event("startup")
async def startup_event():
    """Initialize ROS2 nodes on startup"""
    print("Starting ROS2 Multi-Node System...")
    try:
        initialize_all_nodes()
        print("✅ All ROS2 nodes initialized successfully!")
        print("Available nodes:", list(node_manager.nodes.keys()))
    except Exception as e:
        print(f"❌ Failed to initialize ROS2 nodes: {e}")


@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup ROS2 nodes on shutdown"""
    print("Shutting down ROS2 Multi-Node System...")
    try:
        node_manager.stop_executor()
        print("✅ ROS2 nodes stopped successfully!")
    except Exception as e:
        print(f"❌ Error during shutdown: {e}")


@app.get("/")
async def root():
    return {
        "message": "ROS2 Multi-Node FastAPI Server",
        "version": "1.0.0",
        "endpoints": {
            "docs": "/docs",
            "api": "/api/v1",
            "nodes_status": "/api/v1/nodes/status",
        },
    }


@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return {
        "status": "healthy",
        "ros2_initialized": node_manager.is_running,
        "active_nodes": len(node_manager.nodes),
        "nodes": list(node_manager.nodes.keys()) if node_manager.nodes else [],
    }


def main():
    """Main entry point for the FastAPI server"""
    print("🚀 Starting ROS2 Multi-Node FastAPI Server...")
    uvicorn.run(
        "ros2_fastapi:app",
        host="0.0.0.0",
        port=8000,
        reload=False,  # Don't use reload with ROS2
        log_level="info"
    )


if __name__ == "__main__":
    print("🚀 Starting ROS2 Multi-Node FastAPI Server...")
    uvicorn.run(
        "ros2_fastapi:app",
        host="0.0.0.0",
        port=8000,
        reload=False,  # Don't use reload with ROS2
        log_level="info",
    )
