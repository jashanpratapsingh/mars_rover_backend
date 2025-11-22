# iPad Network Monitoring Setup

This guide shows how to monitor network metrics to your iPad and display them in the frontend.

## Quick Start

### Step 1: Install Dependencies on macOS Host

```bash
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/backend
pip install psutil requests
```

### Step 2: Start Docker Backend

```bash
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/backend
docker-compose up
```

### Step 3: Start Network Monitor for iPad

In a **new terminal**:

```bash
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/backend
python3 network_monitor_host.py 169.254.13.56
```

Or use the default iPad IP:
```bash
python3 network_monitor_host.py
```

### Step 4: Start Joystick Bridge (if using joystick)

In another terminal:
```bash
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/backend
python3 joystick_bridge_host.py
```

### Step 5: Start Frontend

In another terminal:
```bash
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/frontend
npm run dev
```

### Step 6: Open Browser

Open: **http://localhost:5173**

You should see real-time network metrics to your iPad!

## What You'll See

### Backend Console:
```
📡 [HTTP] Network metrics received from host:
   Latency: 2.5ms, Loss: 0.0%, Signal: -45.0dBm
```

### Frontend Dashboard:
- **Latency**: Real ping time to iPad
- **Packet Loss**: Percentage of lost packets
- **Signal Strength**: WiFi signal (if on WiFi)
- **TX/RX Rate**: Data transfer rates
- **Real-time graphs**: Historical data

## iPad IP Address

Your iPad is at: **169.254.13.56** (from ARP table)

To find it again:
```bash
arp -a | grep ipad
```

## Troubleshooting

### "Cannot connect to backend"
- Make sure Docker backend is running: `docker-compose up`
- Check backend is accessible: `curl http://localhost:8765/api/health`

### "Cannot ping iPad"
- Verify iPad is connected to same network
- Try pinging manually: `ping 169.254.13.56`
- Check if iPad IP changed: `arp -a | grep ipad`

### "No metrics showing in frontend"
- Check backend logs for `📡 [HTTP] Network metrics received`
- Verify WebSocket connection shows "CONNECTED" in frontend
- Check browser console (F12) for errors

## Complete Command Sequence

```bash
# Terminal 1: Docker Backend
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/backend
docker-compose up

# Terminal 2: Network Monitor (iPad)
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/backend
python3 network_monitor_host.py 169.254.13.56

# Terminal 3: Joystick Bridge (optional)
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/backend
python3 joystick_bridge_host.py

# Terminal 4: Frontend
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/frontend
npm run dev
```

## Monitoring Different Devices

To monitor a different device, just change the IP:

```bash
# Monitor router
python3 network_monitor_host.py 172.20.10.1

# Monitor iPad
python3 network_monitor_host.py 169.254.13.56

# Monitor another device
python3 network_monitor_host.py <device_ip>
```

## How It Works

1. **Network Monitor Host** (`network_monitor_host.py`) runs on macOS
   - Pings iPad every second
   - Measures latency, packet loss, signal strength
   - Sends metrics via HTTP POST to Docker backend

2. **Docker Backend** receives metrics
   - Stores in ROS node
   - Processes through AI agents
   - Broadcasts via WebSocket to frontend

3. **Frontend** displays metrics
   - Real-time updates via WebSocket
   - Shows latency, packet loss, signal strength
   - Displays historical graphs

Enjoy monitoring your iPad network metrics! 📱📊

