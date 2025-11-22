import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
import random
import time
import threading
import math
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
        self.start_time = time.time()
        self.get_logger().info("Mock Rover Publisher started")
        print("🎮 Mock Joystick Publisher: Generating simulated joystick data...")

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
        
        # Simulate Joystick with dynamic movement
        joy_msg = Joy()
        t = time.time() - self.start_time
        
        # Create smooth circular motion for left stick
        left_x = 0.5 * math.sin(t * 0.5)
        left_y = 0.5 * math.cos(t * 0.5)
        
        # Create different pattern for right stick
        right_x = 0.3 * math.sin(t * 0.7)
        right_y = 0.3 * math.cos(t * 0.9)
        
        # Triggers (L2, R2) - simulate gradual press/release
        l2_axis = 0.5 + 0.5 * math.sin(t * 0.3)
        r2_axis = 0.5 + 0.5 * math.cos(t * 0.4)
        
        joy_msg.axes = [
            left_x,      # 0: Left stick X
            left_y,      # 1: Left stick Y
            l2_axis,     # 2: L2 trigger
            right_x,      # 3: Right stick X
            right_y,      # 4: Right stick Y
            r2_axis      # 5: R2 trigger
        ]
        
        # Buttons - occasionally press random buttons
        buttons = [0] * 13
        if random.random() < 0.1:  # 10% chance to press a button
            buttons[random.randint(0, 12)] = 1
        
        joy_msg.buttons = buttons
        self.joy_pub.publish(joy_msg)
        
        # Log occasionally
        if int(t) % 5 == 0 and t - int(t) < 0.1:
            print(f"🎮 [MOCK] Published joystick - L:({left_x:.2f},{left_y:.2f}) R:({right_x:.2f},{right_y:.2f})")

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
