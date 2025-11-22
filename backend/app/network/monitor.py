"""
Network Monitor Module
Monitors network communication with a device on the local network.
Measures latency, packet loss, signal strength, and data rates.
"""
import subprocess
import platform
import time
import statistics
import threading
from typing import Dict, Any, Optional, Callable
import logging
import socket
import psutil

logger = logging.getLogger(__name__)


class NetworkMonitor:
    """
    Monitors network metrics by pinging a target device and measuring network stats.
    """
    
    def __init__(
        self,
        target_host: str = "192.168.1.1",
        ping_interval: float = 1.0,
        ping_count: int = 4,
        callback: Optional[Callable[[Dict[str, Any]], None]] = None
    ):
        """
        Initialize network monitor.
        
        Args:
            target_host: IP address or hostname to monitor (default: router)
            ping_interval: Time between ping batches (seconds)
            ping_count: Number of pings per batch
            callback: Function to call with metrics dict
        """
        self.target_host = target_host
        self.ping_interval = ping_interval
        self.ping_count = ping_count
        self.callback = callback
        self.running = False
        self.monitor_thread: Optional[threading.Thread] = None
        
        # Network interface detection
        self.network_interface = self._detect_network_interface()
        
        # Statistics
        self.latency_history: list = []
        self.packet_loss_history: list = []
        self.max_history_size = 100
        
        logger.info(f"Network Monitor initialized for target: {target_host}")
        logger.info(f"Network interface: {self.network_interface}")
    
    def _detect_network_interface(self) -> Optional[str]:
        """Detect the active network interface."""
        try:
            # Get default gateway interface
            if platform.system() == "Darwin":  # macOS
                result = subprocess.run(
                    ["route", "get", "default"],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                for line in result.stdout.split('\n'):
                    if 'interface:' in line:
                        return line.split(':')[1].strip()
            elif platform.system() == "Linux":
                result = subprocess.run(
                    ["ip", "route", "show", "default"],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                for line in result.stdout.split('\n'):
                    if 'dev' in line:
                        parts = line.split()
                        if 'dev' in parts:
                            idx = parts.index('dev')
                            if idx + 1 < len(parts):
                                return parts[idx + 1]
        except Exception as e:
            logger.warning(f"Could not detect network interface: {e}")
        
        return None
    
    def _get_signal_strength(self) -> float:
        """
        Get WiFi signal strength in dBm.
        Returns -90 if not available or not WiFi.
        """
        try:
            if platform.system() == "Darwin":  # macOS
                result = subprocess.run(
                    ["/System/Library/PrivateFrameworks/Apple80211.framework/Versions/Current/Resources/airport", "-I"],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                for line in result.stdout.split('\n'):
                    if 'agrCtlRSSI' in line or 'RSSI' in line:
                        try:
                            rssi = float(line.split(':')[1].strip())
                            return rssi
                        except (ValueError, IndexError):
                            pass
            elif platform.system() == "Linux":
                # Try iwconfig
                result = subprocess.run(
                    ["iwconfig"],
                    capture_output=True,
                    text=True,
                    timeout=2
                )
                for line in result.stdout.split('\n'):
                    if 'Signal level' in line:
                        try:
                            # Format: "Signal level=-50 dBm"
                            level_str = line.split('Signal level=')[1].split()[0]
                            return float(level_str)
                        except (ValueError, IndexError):
                            pass
        except Exception as e:
            logger.debug(f"Could not get signal strength: {e}")
        
        # Default: assume wired connection or unknown
        return -90.0
    
    def _ping_host(self) -> tuple:
        """
        Ping the target host and return (average_latency_ms, packet_loss_percent).
        
        Returns:
            (latency_ms, packet_loss_percent)
        """
        try:
            if platform.system() == "Darwin":  # macOS
                cmd = ["ping", "-c", str(self.ping_count), "-W", "1000", self.target_host]
            elif platform.system() == "Linux":
                cmd = ["ping", "-c", str(self.ping_count), "-W", "1", self.target_host]
            else:  # Windows
                cmd = ["ping", "-n", str(self.ping_count), "-w", "1000", self.target_host]
            
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=self.ping_count * 2 + 2
            )
            
            if result.returncode != 0:
                # Host unreachable or timeout
                return (999.0, 100.0)
            
            # Parse ping output
            output = result.stdout
            
            # Extract packet loss
            packet_loss = 0.0
            for line in output.split('\n'):
                if 'packet loss' in line.lower() or '% packet loss' in line:
                    try:
                        # Format: "3 packets transmitted, 3 received, 0% packet loss"
                        parts = line.split('%')
                        if len(parts) > 0:
                            packet_loss_str = parts[0].split()[-1]
                            packet_loss = float(packet_loss_str)
                    except (ValueError, IndexError):
                        pass
            
            # Extract latency (average)
            latency = 0.0
            latencies = []
            for line in output.split('\n'):
                if 'time=' in line or 'time<' in line:
                    try:
                        # Format: "64 bytes from 192.168.1.1: icmp_seq=0 ttl=64 time=2.345 ms"
                        time_part = line.split('time=')[1].split()[0] if 'time=' in line else line.split('time<')[1].split()[0]
                        latency_ms = float(time_part)
                        latencies.append(latency_ms)
                    except (ValueError, IndexError):
                        pass
            
            if latencies:
                latency = statistics.mean(latencies)
            else:
                # Fallback: try to parse min/avg/max line
                for line in output.split('\n'):
                    if 'min/avg/max' in line.lower() or '/avg/' in line:
                        try:
                            # Format: "round-trip min/avg/max = 1.234/2.345/3.456 ms"
                            parts = line.split('/')
                            if len(parts) >= 3:
                                latency = float(parts[1])
                        except (ValueError, IndexError):
                            pass
            
            return (latency, packet_loss)
            
        except subprocess.TimeoutExpired:
            logger.warning(f"Ping timeout for {self.target_host}")
            return (999.0, 100.0)
        except Exception as e:
            logger.error(f"Error pinging {self.target_host}: {e}")
            return (999.0, 100.0)
    
    def _get_network_stats(self) -> tuple:
        """
        Get current network TX and RX rates in Mbps.
        
        Returns:
            (tx_rate_mbps, rx_rate_mbps)
        """
        try:
            net_io = psutil.net_io_counters(pernic=True)
            
            if self.network_interface and self.network_interface in net_io:
                stats = net_io[self.network_interface]
                # Get bytes per second (approximate)
                # Note: This is cumulative, so we'd need to track previous values
                # For now, return a simple estimate
                tx_bytes = stats.bytes_sent
                rx_bytes = stats.bytes_recv
                
                # Convert to Mbps (rough estimate, would need time delta for accurate rate)
                # This is a simplified version - for real rates, need to track over time
                return (0.0, 0.0)  # Placeholder - would need rate calculation
            
            # Fallback: use total network stats
            total_io = psutil.net_io_counters()
            return (0.0, 0.0)  # Placeholder
            
        except Exception as e:
            logger.debug(f"Could not get network stats: {e}")
            return (0.0, 0.0)
    
    def _monitor_loop(self):
        """Main monitoring loop running in background thread."""
        last_stats_time = time.time()
        last_tx_bytes = 0
        last_rx_bytes = 0
        
        while self.running:
            try:
                # Ping target host
                latency, packet_loss = self._ping_host()
                
                # Update history
                self.latency_history.append(latency)
                if len(self.latency_history) > self.max_history_size:
                    self.latency_history.pop(0)
                
                self.packet_loss_history.append(packet_loss)
                if len(self.packet_loss_history) > self.max_history_size:
                    self.packet_loss_history.pop(0)
                
                # Get signal strength
                signal_strength = self._get_signal_strength()
                
                # Calculate network rates (simplified - would need proper rate tracking)
                current_time = time.time()
                time_delta = current_time - last_stats_time
                
                tx_rate = 0.0
                rx_rate = 0.0
                
                try:
                    net_io = psutil.net_io_counters(pernic=True)
                    if self.network_interface and self.network_interface in net_io:
                        stats = net_io[self.network_interface]
                        if time_delta > 0:
                            tx_rate = ((stats.bytes_sent - last_tx_bytes) * 8) / (time_delta * 1_000_000)  # Mbps
                            rx_rate = ((stats.bytes_recv - last_rx_bytes) * 8) / (time_delta * 1_000_000)  # Mbps
                        
                        last_tx_bytes = stats.bytes_sent
                        last_rx_bytes = stats.bytes_recv
                except Exception:
                    pass
                
                last_stats_time = current_time
                
                # Create metrics dict
                metrics = {
                    "latency": latency,
                    "packet_loss": packet_loss,
                    "signal_strength": signal_strength,
                    "tx_rate": tx_rate,
                    "rx_rate": rx_rate,
                    "timestamp": time.time()
                }
                
                # Call callback if provided
                if self.callback:
                    try:
                        self.callback(metrics)
                    except Exception as e:
                        logger.error(f"Error in network monitor callback: {e}")
                
                # Log occasionally
                if int(time.time()) % 10 == 0:
                    logger.info(
                        f"📡 Network: {latency:.1f}ms latency, "
                        f"{packet_loss:.1f}% loss, "
                        f"{signal_strength:.1f}dBm signal"
                    )
                
            except Exception as e:
                logger.error(f"Error in network monitor loop: {e}")
            
            # Sleep until next ping interval
            time.sleep(self.ping_interval)
    
    def start(self):
        """Start the network monitor in a background thread."""
        if self.running:
            logger.warning("Network monitor is already running")
            return
        
        self.running = True
        self.monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.monitor_thread.start()
        logger.info(f"✅ Network monitor started for {self.target_host}")
    
    def stop(self):
        """Stop the network monitor."""
        self.running = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2.0)
        logger.info("Network monitor stopped")
    
    def get_latest_metrics(self) -> Optional[Dict[str, Any]]:
        """Get the latest network metrics."""
        if not self.latency_history:
            return None
        
        return {
            "latency": self.latency_history[-1] if self.latency_history else 0.0,
            "packet_loss": self.packet_loss_history[-1] if self.packet_loss_history else 0.0,
            "signal_strength": self._get_signal_strength(),
            "tx_rate": 0.0,  # Would need proper rate tracking
            "rx_rate": 0.0,  # Would need proper rate tracking
            "timestamp": time.time()
        }

