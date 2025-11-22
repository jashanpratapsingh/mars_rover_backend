# Network Monitor Setup Guide

This guide explains how to set up real-time network monitoring between your computer and a device on the local network.

## Overview

The Network Monitor continuously measures network metrics by:
- **Pinging** a target device on your local network
- **Measuring latency** (round-trip time)
- **Calculating packet loss** percentage
- **Detecting WiFi signal strength** (if on WiFi)
- **Tracking data rates** (TX/RX in Mbps)

All metrics are displayed in real-time on the frontend dashboard.

## Quick Start

### Step 1: Find Your Target Device IP

First, identify the IP address of the device you want to monitor:

**On macOS/Linux:**
```bash
# Find devices on your network
arp -a

# Or scan your network
nmap -sn 192.168.1.0/24
```

**Common targets:**
- Router/Gateway: Usually `192.168.1.1` or `192.168.0.1`
- Raspberry Pi: Check your router's connected devices
- Another computer: Use `ipconfig` (Windows) or `ifconfig` (macOS/Linux)

### Step 2: Start Backend with Network Monitor

**Option A: Direct Python**
```bash
cd backend
python3 main.py --network-monitor --network-target 192.168.1.100
```

**Option B: Docker**
```bash
cd backend
# Set environment variables
export USE_NETWORK_MONITOR=1
export NETWORK_TARGET_HOST=192.168.1.100
export NETWORK_PING_INTERVAL=1.0

docker-compose build
docker-compose up
```

### Step 3: Verify It's Working

1. **Check backend logs** - You should see:
   ```
   📡 Starting Network Monitor for 192.168.1.100...
   ✅ Network Monitor started - Monitoring 192.168.1.100
   📡 Network: 15.2ms latency, 0.0% loss, -45.0dBm signal
   ```

2. **Check frontend** - Open the dashboard and verify:
   - Latency shows real values (not mock data)
   - Packet Loss is measured
   - Signal Strength shows WiFi signal (if on WiFi)

## Command Line Options

```bash
python3 main.py --network-monitor                    # Enable network monitor
python3 main.py --network-monitor --network-target 192.168.1.100  # Specify target
python3 main.py --network-monitor --network-target 192.168.1.100 --network-ping-interval 2.0  # Custom ping interval
```

### Options:
- `--network-monitor`: Enable real network monitoring
- `--network-target <IP>`: Target device IP/hostname (default: 192.168.1.1)
- `--network-ping-interval <seconds>`: Time between ping batches (default: 1.0)

## Environment Variables

You can also use environment variables:

```bash
export USE_NETWORK_MONITOR=1
export NETWORK_TARGET_HOST=192.168.1.100
export NETWORK_PING_INTERVAL=1.0

python3 main.py
```

## How It Works

### Architecture

```
Network Monitor
    ↓ (pings target device)
Measures: Latency, Packet Loss, Signal Strength, Data Rates
    ↓ (publishes to ROS topic)
ROS Node (subscribes to network_metrics)
    ↓ (processes data)
Backend Server
    ↓ (broadcasts via WebSocket)
Frontend Dashboard (displays real-time metrics)
```

### Metrics Explained

1. **Latency (ms)**: Round-trip time for ping packets
   - Good: < 50ms
   - Warning: 50-100ms
   - Critical: > 100ms

2. **Packet Loss (%)**: Percentage of lost ping packets
   - Good: < 1%
   - Warning: 1-3%
   - Critical: > 3%

3. **Signal Strength (dBm)**: WiFi signal strength (if on WiFi)
   - Excellent: > -50 dBm
   - Good: -50 to -65 dBm
   - Fair: -65 to -75 dBm
   - Poor: < -75 dBm

4. **TX/RX Rate (Mbps)**: Data transfer rates
   - Currently shows 0.0 (requires additional tracking)

## Troubleshooting

### "Host unreachable" or High Latency

1. **Check target device is on same network**:
   ```bash
   ping 192.168.1.100
   ```

2. **Verify firewall isn't blocking ICMP**:
   - Some devices block ping by default
   - Check router/firewall settings

3. **Try different target**:
   - Use your router IP (usually `192.168.1.1`)
   - Or another device you know is online

### No Signal Strength Reading

- **Wired connection**: Signal strength will show -90 dBm (default)
- **WiFi on macOS**: Requires airport command (should work automatically)
- **WiFi on Linux**: Requires `iwconfig` (install: `sudo apt install wireless-tools`)

### High Packet Loss

- Check network congestion
- Verify target device is stable
- Check for interference (if WiFi)
- Try increasing ping interval: `--network-ping-interval 2.0`

### Network Monitor Not Starting

1. **Check psutil is installed**:
   ```bash
   pip install psutil
   ```

2. **Check permissions**:
   - Network monitoring requires ping permissions
   - On Linux, may need to run with appropriate permissions

3. **Check logs**:
   ```bash
   # Look for error messages in backend output
   python3 main.py --network-monitor --network-target 192.168.1.1
   ```

## Advanced Usage

### Monitor Multiple Devices

Currently, the monitor tracks one device. To monitor multiple:
1. Run multiple backend instances (different ports)
2. Or modify the code to ping multiple targets

### Custom Ping Settings

Adjust ping count and timeout:
- Edit `backend/app/network/monitor.py`
- Modify `ping_count` in `NetworkMonitor.__init__()`

### Integration with Real Rover

When connected to a real rover:
1. Set rover's IP as target: `--network-target <rover-ip>`
2. Monitor connection quality in real-time
3. Get alerts when latency/packet loss is high

## Example: Monitoring a Raspberry Pi

```bash
# 1. Find Raspberry Pi IP (usually shown on boot or check router)
# Example: 192.168.1.50

# 2. Start backend with network monitor
cd backend
python3 main.py --network-monitor --network-target 192.168.1.50

# 3. Open frontend dashboard
# You'll see real-time network metrics to your Raspberry Pi!
```

## Integration with Existing System

The network monitor:
- ✅ Automatically disables mock publisher when active
- ✅ Publishes to same ROS topic as mock data
- ✅ Works with existing frontend (no changes needed)
- ✅ Integrates with AI agents (anomaly detection, prediction)
- ✅ Logs to database (same as mock data)

## Next Steps

1. **Test with your router**: `--network-target 192.168.1.1`
2. **Monitor your rover device**: Set rover IP as target
3. **Watch the dashboard**: See real-time network health
4. **Get alerts**: System will alert on high latency/packet loss

Enjoy real-time network monitoring! 🚀

