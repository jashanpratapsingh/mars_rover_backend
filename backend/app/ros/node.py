import sys
import os
import threading
import time
from typing import Dict, Any, Optional
from app.core.config import settings

# Try importing rclpy, if fails, inject mocks
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
except ImportError:
    print("ROS 2 (rclpy) not found. Injecting mocks...")
    # Calculate path to mocks: ../../mocks relative to this file
    current_dir = os.path.dirname(os.path.abspath(__file__))
    mocks_path = os.path.abspath(os.path.join(current_dir, "..", "..", "mocks"))
    if mocks_path not in sys.path:
        sys.path.insert(0, mocks_path)
    
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy

class TascMonitoringBackendNode(Node):
    def __init__(self):
        super().__init__(settings.ros.node_name)
        
        # Thread-safe storage for latest data
        self._lock = threading.Lock()
        self.latest_network_metrics: Optional[Dict[str, Any]] = None
        self.latest_joystick_data: Optional[Dict[str, Any]] = None
        self.last_metrics_time = 0.0
        
        # QoS Profile: Best Effort, Depth 10
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=settings.ros.qos_history_depth
        )
        
        # Subscriptions
        self.metrics_sub = self.create_subscription(
            Float32MultiArray,
            settings.ros.topics['network_metrics'],
            self._metrics_callback,
            qos_profile
        )
        
        self.joy_sub = self.create_subscription(
            Joy,
            settings.ros.topics['joy'],
            self._joy_callback,
            qos_profile
        )
        
        self.get_logger().info(f"Node {settings.ros.node_name} started. Subscribed to topics.")

    def _metrics_callback(self, msg: Float32MultiArray):
        """
        Callback for network metrics.
        Expected format: [latency, packet_loss, signal_strength, tx_rate, rx_rate]
        """
        current_time = time.time()
        
        if len(msg.data) < 5:
            self.get_logger().warn("Received incomplete network metrics")
            return

        with self._lock:
            self.latest_network_metrics = {
                "latency": msg.data[0],
                "packet_loss": msg.data[1],
                "signal_strength": msg.data[2],
                "tx_rate": msg.data[3],
                "rx_rate": msg.data[4],
                "timestamp": current_time
            }
            self.last_metrics_time = current_time

    def _joy_callback(self, msg: Joy):
        """
        Callback for joystick data.
        """
        axes_list = list(msg.axes)
        buttons_list = list(msg.buttons)
        
        # Log joystick data to console
        print(f"\n🎮 [ROS] Joystick received - Axes: {[f'{a:.2f}' for a in axes_list]}, Buttons: {buttons_list}")
        
        with self._lock:
            self.latest_joystick_data = {
                "axes": axes_list,
                "buttons": buttons_list,
                "timestamp": time.time()
            }

    def get_latest_metrics(self) -> Optional[Dict[str, Any]]:
        with self._lock:
            if self.latest_network_metrics:
                return self.latest_network_metrics.copy()
            return None

    def get_latest_joystick(self) -> Optional[Dict[str, Any]]:
        with self._lock:
            if self.latest_joystick_data:
                return self.latest_joystick_data.copy()
            return None

def start_ros_node():
    """
    Helper to start the ROS node in a separate thread.
    Returns the node instance and the thread.
    """
    rclpy.init()
    node = TascMonitoringBackendNode()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    return node, thread
