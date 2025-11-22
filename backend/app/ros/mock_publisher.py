import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
import random
import time
import threading
from app.core.config import settings

class MockRoverPublisher(Node):
    def __init__(self):
        super().__init__('mock_rover_publisher')
        
        self.metrics_pub = self.create_publisher(
            Float32MultiArray,
            settings.ros.topics['network_metrics'],
            10
        )
        
        self.joy_pub = self.create_publisher(
            Joy,
            settings.ros.topics['joy'],
            10
        )
        
        self.timer = self.create_timer(1.0 / settings.system.update_rate_hz, self.publish_data)
        self.get_logger().info("Mock Rover Publisher started")

    def publish_data(self):
        # Simulate Network Metrics
        # [latency, packet_loss, signal_strength, tx_rate, rx_rate]
        metrics_msg = Float32MultiArray()
        metrics_msg.data = [
            random.uniform(20.0, 80.0),   # Latency (ms)
            random.uniform(0.0, 1.0),     # Packet Loss (%)
            random.uniform(-60.0, -40.0), # Signal Strength (dBm)
            random.uniform(100.0, 150.0), # TX Rate (Mbps)
            random.uniform(80.0, 120.0)   # RX Rate (Mbps)
        ]
        self.metrics_pub.publish(metrics_msg)
        
        # Simulate Joystick
        joy_msg = Joy()
        joy_msg.axes = [random.uniform(-1.0, 1.0) for _ in range(6)]
        joy_msg.buttons = [random.randint(0, 1) for _ in range(12)]
        self.joy_pub.publish(joy_msg)

def start_mock_publisher():
    # Note: rclpy.init() should be called globally before this if running in same process
    # But usually this is run as a separate script.
    # For this project, we might run it in the same process for testing if needed,
    # but ideally it's a separate process.
    node = MockRoverPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

if __name__ == "__main__":
    rclpy.init()
    start_mock_publisher()
    rclpy.shutdown()
