"""
Network Metrics Publisher
Publishes real network metrics to ROS 2 topic.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from app.network.monitor import NetworkMonitor
from app.core.config import settings
import logging

logger = logging.getLogger(__name__)


class NetworkMetricsPublisher(Node):
    """
    ROS 2 node that publishes network metrics from NetworkMonitor.
    """
    
    def __init__(self, target_host: str = "192.168.1.1", ping_interval: float = 1.0):
        super().__init__('network_metrics_publisher')
        
        self.metrics_pub = self.create_publisher(
            Float32MultiArray,
            settings.ros.topics['network_metrics'],
            10
        )
        
        # Create network monitor
        self.network_monitor = NetworkMonitor(
            target_host=target_host,
            ping_interval=ping_interval,
            callback=self._on_metrics_received
        )
        
        self.get_logger().info(f"Network Metrics Publisher initialized for {target_host}")
        print(f"📡 Network Monitor: Monitoring {target_host}")
    
    def _on_metrics_received(self, metrics: dict):
        """Callback when network monitor receives new metrics."""
        try:
            # Create ROS message
            msg = Float32MultiArray()
            msg.data = [
                float(metrics['latency']),
                float(metrics['packet_loss']),
                float(metrics['signal_strength']),
                float(metrics['tx_rate']),
                float(metrics['rx_rate'])
            ]
            
            # Publish to ROS topic
            self.metrics_pub.publish(msg)
            
            # Log occasionally
            if int(metrics['timestamp']) % 10 == 0:
                self.get_logger().info(
                    f"Published metrics: {metrics['latency']:.1f}ms, "
                    f"{metrics['packet_loss']:.1f}% loss"
                )
                
        except Exception as e:
            logger.error(f"Error publishing network metrics: {e}")
    
    def start(self):
        """Start the network monitor."""
        self.network_monitor.start()
        self.get_logger().info("Network monitor started")
    
    def stop(self):
        """Stop the network monitor."""
        self.network_monitor.stop()
        self.get_logger().info("Network monitor stopped")


def start_network_publisher(target_host: str = "192.168.1.1", ping_interval: float = 1.0):
    """
    Start network metrics publisher in a separate thread.
    
    Args:
        target_host: IP address or hostname to monitor
        ping_interval: Time between ping batches (seconds)
    
    Returns:
        (publisher, thread) tuple
    """
    import threading
    
    def run_publisher():
        try:
            rclpy.init()
            publisher = NetworkMetricsPublisher(target_host, ping_interval)
            publisher.start()
            
            # Spin the node
            rclpy.spin(publisher)
        except KeyboardInterrupt:
            pass
        except Exception as e:
            logger.error(f"Error in network publisher: {e}")
        finally:
            if 'publisher' in locals():
                publisher.stop()
            rclpy.shutdown()
    
    thread = threading.Thread(target=run_publisher, daemon=True)
    thread.start()
    
    return thread

