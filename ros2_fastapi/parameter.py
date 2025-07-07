from typing import Any
from datetime import datetime
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
import rclpy
from dataclasses import dataclass


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
