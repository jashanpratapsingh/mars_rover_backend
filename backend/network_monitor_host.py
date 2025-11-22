#!/usr/bin/env python3
"""
Network Monitor Bridge for macOS Host
Monitors network metrics to a device (like iPad) and sends to Docker container via HTTP POST.
Run this on your Mac while the backend runs in Docker.

Install dependencies:
    pip install psutil requests
"""
import subprocess
import platform
import time
import statistics
import requests
import sys
import psutil

def ping_host(target_host, ping_count=4):
    """Ping target and return (latency_ms, packet_loss_percent)"""
    try:
        if platform.system() == "Darwin":  # macOS
            cmd = ["ping", "-c", str(ping_count), "-W", "1000", target_host]
        elif platform.system() == "Linux":
            cmd = ["ping", "-c", str(ping_count), "-W", "1", target_host]
        else:  # Windows
            cmd = ["ping", "-n", str(ping_count), "-w", "1000", target_host]
        
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=ping_count * 2 + 2
        )
        
        if result.returncode != 0:
            return (999.0, 100.0)
        
        output = result.stdout
        
        # Extract packet loss
        packet_loss = 0.0
        for line in output.split('\n'):
            if 'packet loss' in line.lower() or '% packet loss' in line:
                try:
                    parts = line.split('%')
                    if len(parts) > 0:
                        packet_loss_str = parts[0].split()[-1]
                        packet_loss = float(packet_loss_str)
                except (ValueError, IndexError):
                    pass
        
        # Extract latency
        latency = 0.0
        latencies = []
        for line in output.split('\n'):
            if 'time=' in line or 'time<' in line:
                try:
                    time_part = line.split('time=')[1].split()[0] if 'time=' in line else line.split('time<')[1].split()[0]
                    latency_ms = float(time_part)
                    latencies.append(latency_ms)
                except (ValueError, IndexError):
                    pass
        
        if latencies:
            latency = statistics.mean(latencies)
        else:
            for line in output.split('\n'):
                if 'min/avg/max' in line.lower() or '/avg/' in line:
                    try:
                        parts = line.split('/')
                        if len(parts) >= 3:
                            latency = float(parts[1])
                    except (ValueError, IndexError):
                        pass
        
        return (latency, packet_loss)
        
    except Exception as e:
        print(f"Error pinging {target_host}: {e}")
        return (999.0, 100.0)

def get_signal_strength():
    """Get WiFi signal strength in dBm"""
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
    except Exception:
        pass
    
    return -90.0  # Default

def main():
    # Get target from command line or use iPad IP
    if len(sys.argv) > 1:
        target_host = sys.argv[1]
    else:
        # Default to iPad IP from ARP table
        target_host = "169.254.13.56"
        print("💡 Usage: python3 network_monitor_host.py <target_ip>")
        print(f"   Using default iPad IP: {target_host}")
    
    print("📡 Network Monitor Bridge for macOS Host")
    print("=" * 60)
    print(f"Monitoring: {target_host}")
    print("Sending metrics to Docker backend at http://localhost:8765")
    print("=" * 60)
    print()
    
    # Test backend connection
    print("🔍 Testing backend connection...")
    try:
        test_response = requests.get("http://localhost:8765/api/health", timeout=2)
        if test_response.status_code == 200:
            print("✅ Backend is ready!")
        else:
            print("⚠️  Backend responded but with unexpected status")
    except requests.exceptions.ConnectionError:
        print("❌ Cannot connect to backend!")
        print("   Make sure Docker backend is running:")
        print("   cd backend && docker-compose up")
        sys.exit(1)
    except Exception as e:
        print(f"⚠️  Error testing backend: {e}")
    print()
    
    backend_url = "http://localhost:8765/api/network/update"
    
    try:
        # Initialize rate tracking
        last_stats_time = time.time()
        last_tx_bytes = {}
        last_rx_bytes = {}
        last_update_time = time.time()
        error_count = 0
        success_count = 0
        
        # Get initial stats for all interfaces
        try:
            initial_net_io = psutil.net_io_counters(pernic=True)
            for interface, stats in initial_net_io.items():
                if interface.startswith('en') or interface.startswith('wlan') or interface.startswith('eth'):
                    last_tx_bytes[interface] = stats.bytes_sent
                    last_rx_bytes[interface] = stats.bytes_recv
        except Exception:
            pass
        
        print("📊 Starting network monitoring...")
        print("   Press Ctrl+C to stop")
        print()
        
        while True:
            # Ping target
            latency, packet_loss = ping_host(target_host, ping_count=4)
            
            # Get signal strength
            signal_strength = get_signal_strength()
            
            # Calculate network rates from all active interfaces
            current_time = time.time()
            time_delta = current_time - last_stats_time
            
            tx_rate = 0.0
            rx_rate = 0.0
            
            try:
                net_io = psutil.net_io_counters(pernic=True)
                
                # Sum rates from all active interfaces
                total_tx_delta = 0
                total_rx_delta = 0
                
                for interface, stats in net_io.items():
                    # Only track physical network interfaces (skip loopback, etc.)
                    if (interface.startswith('en') or interface.startswith('wlan') or 
                        interface.startswith('eth') or interface.startswith('wifi')):
                        
                        if interface in last_tx_bytes and time_delta > 0:
                            # Calculate rate for this interface
                            tx_delta = stats.bytes_sent - last_tx_bytes[interface]
                            rx_delta = stats.bytes_recv - last_rx_bytes[interface]
                            
                            # Convert bytes to bits and then to Mbps
                            tx_rate_interface = (tx_delta * 8) / (time_delta * 1_000_000)
                            rx_rate_interface = (rx_delta * 8) / (time_delta * 1_000_000)
                            
                            # Sum all interface rates
                            total_tx_delta += tx_delta
                            total_rx_delta += rx_delta
                            
                            # Update stored values
                            last_tx_bytes[interface] = stats.bytes_sent
                            last_rx_bytes[interface] = stats.bytes_recv
                        else:
                            # First time seeing this interface
                            last_tx_bytes[interface] = stats.bytes_sent
                            last_rx_bytes[interface] = stats.bytes_recv
                
                # Calculate total rates
                if time_delta > 0:
                    tx_rate = (total_tx_delta * 8) / (time_delta * 1_000_000)  # Mbps
                    rx_rate = (total_rx_delta * 8) / (time_delta * 1_000_000)  # Mbps
                
            except Exception as e:
                # If error, try to get total network stats as fallback
                try:
                    total_io = psutil.net_io_counters()
                    if 'total_tx_bytes' not in locals():
                        total_tx_bytes = total_io.bytes_sent
                        total_rx_bytes = total_io.bytes_recv
                    else:
                        if time_delta > 0:
                            tx_rate = ((total_io.bytes_sent - total_tx_bytes) * 8) / (time_delta * 1_000_000)
                            rx_rate = ((total_io.bytes_recv - total_rx_bytes) * 8) / (time_delta * 1_000_000)
                        total_tx_bytes = total_io.bytes_sent
                        total_rx_bytes = total_io.bytes_recv
                except Exception:
                    pass
            
            last_stats_time = current_time
            
            # Send to backend
            try:
                response = requests.post(
                    backend_url,
                    json={
                        "latency": latency,
                        "packet_loss": packet_loss,
                        "signal_strength": signal_strength,
                        "tx_rate": tx_rate,
                        "rx_rate": rx_rate,
                        "timestamp": current_time
                    },
                    timeout=0.5
                )
                
                if response.status_code == 200:
                    success_count += 1
                    error_count = 0
                    
                    if success_count == 1 or success_count % 30 == 0:
                        print(f"✅ Sent metrics: {latency:.1f}ms latency, {packet_loss:.1f}% loss, {signal_strength:.1f}dBm")
                        print(f"   TX: {tx_rate:.2f} Mbps, RX: {rx_rate:.2f} Mbps")
                else:
                    error_count += 1
                    if error_count == 1:
                        print(f"⚠️  Backend returned status {response.status_code}")
            except requests.exceptions.ConnectionError:
                error_count += 1
                if error_count == 1 or error_count % 30 == 0:
                    print("   ⏳ Cannot connect to backend - is Docker running?")
            except Exception as e:
                error_count += 1
                if error_count == 1:
                    print(f"⚠️  Error sending metrics: {e}")
            
            time.sleep(1.0)  # 1 Hz update rate
            
    except KeyboardInterrupt:
        print("\n✅ Stopped")

if __name__ == "__main__":
    main()

