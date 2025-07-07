from datetime import datetime
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from dataclasses import dataclass


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
