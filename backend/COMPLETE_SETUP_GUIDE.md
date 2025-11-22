# Complete Setup Guide: Network Monitor + Joystick

This guide covers setting up both **Network Monitor** and **Joystick** components.

---

## 🎮 Part 1: Joystick Setup

### Option A: Using Docker (Recommended for macOS)

#### Step 1: Install pygame on macOS host
```bash
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/backend
pip install pygame requests
```

#### Step 2: Start Docker backend
```bash
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/backend
./run_docker.sh
# OR
docker-compose up
```

#### Step 3: Connect PS4 controller
1. Put controller in pairing mode: Hold **SHARE** + **PS** buttons
2. Pair with Mac: System Preferences > Bluetooth > "Wireless Controller"
3. Verify connection: Controller light should be solid

#### Step 4: Start joystick bridge (in new terminal)
```bash
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/backend
python3 joystick_bridge_host.py
```

You should see:
```
✅ Found 1 joystick(s)
✅ Connected: Wireless Controller
✅ Sent joystick data to backend
```

### Option B: Direct Python (Linux/Windows)

#### Step 1: Install dependencies
```bash
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/backend
pip install pygame
```

#### Step 2: Connect PS4 controller
- **Bluetooth**: Pair via system settings
- **USB**: Plug in directly

#### Step 3: Start backend with joystick
```bash
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/backend
python3 main.py
```

The system will automatically detect and use your PS4 controller!

---

## 📡 Part 2: Network Monitor Setup

### Step 1: Install psutil
```bash
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/backend
pip install psutil
```

### Step 2: Find target device IP
```bash
# Test common router IPs
ping 192.168.1.1
# OR
ping 192.168.0.1

# If ping works, use that IP
# Otherwise, find your router IP from network settings
```

### Step 3: Start backend with network monitor
```bash
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/backend
python3 main.py --network-monitor --network-target 192.168.1.1
```

Replace `192.168.1.1` with your target device IP.

---

## 🚀 Complete Setup: Both Joystick + Network Monitor

### For macOS with Docker:

#### Terminal 1: Backend with Network Monitor
```bash
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/backend

# Install dependencies
pip install psutil

# Start Docker backend
docker-compose up
```

#### Terminal 2: Joystick Bridge
```bash
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/backend

# Install pygame if not already installed
pip install pygame requests

# Start joystick bridge
python3 joystick_bridge_host.py
```

#### Terminal 3: Frontend
```bash
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/frontend
npm run dev
```

#### Terminal 4: Start Network Monitor (if not using Docker)
```bash
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/backend
python3 main.py --network-monitor --network-target 192.168.1.1
```

**Note**: If using Docker, you need to set environment variables for network monitor:
```bash
export USE_NETWORK_MONITOR=1
export NETWORK_TARGET_HOST=192.168.1.1
docker-compose up
```

### For Linux/Windows (Direct Python):

#### Terminal 1: Backend with Everything
```bash
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/backend

# Install all dependencies
pip install pygame psutil

# Start backend with network monitor
python3 main.py --network-monitor --network-target 192.168.1.1
```

#### Terminal 2: Frontend
```bash
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/frontend
npm run dev
```

---

## ✅ Verification Checklist

### Joystick Working:
- [ ] Controller connected (solid light)
- [ ] Backend shows: `🎮 [HTTP] Joystick update received` (Docker) or `🎮 [ROS] Joystick received` (Direct)
- [ ] Frontend shows joystick visualization updating
- [ ] Moving sticks updates frontend in real-time
- [ ] Button presses show in frontend

### Network Monitor Working:
- [ ] Backend shows: `📡 Network Monitor started - Monitoring <IP>`
- [ ] Backend shows: `📡 Network: X.Xms latency, X.X% loss`
- [ ] Frontend shows real latency values (not mock)
- [ ] Frontend shows packet loss percentage
- [ ] Frontend shows signal strength (if WiFi)

---

## 🔧 Troubleshooting

### Joystick Issues:

**"No joysticks detected"**
```bash
# Test pygame detection
python3 -c "import pygame; pygame.init(); pygame.joystick.init(); print(f'Found {pygame.joystick.get_count()} joystick(s)')"
```

**"Controller not showing in frontend"**
- Check backend logs for `🎮 [BROADCAST] Joystick Update`
- Check frontend console (F12) for `🎮 Joystick update received`
- Verify WebSocket connection shows "CONNECTED"

**"Docker can't detect joystick"**
- This is expected! Use `joystick_bridge_host.py` on macOS
- Make sure bridge script is running in separate terminal

### Network Monitor Issues:

**"Host unreachable"**
```bash
# Test ping manually
ping 192.168.1.1

# Try different target
python3 main.py --network-monitor --network-target 192.168.0.1
```

**"psutil not found"**
```bash
pip install psutil
```

**"No metrics showing"**
- Check backend logs for `📡 Network:` messages
- Verify target IP is correct
- Check firewall isn't blocking ping

---

## 📋 Quick Reference Commands

### Install All Dependencies:
```bash
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/backend
pip install pygame psutil requests
```

### Start Everything (macOS Docker):
```bash
# Terminal 1: Docker backend
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/backend
export USE_NETWORK_MONITOR=1
export NETWORK_TARGET_HOST=192.168.1.1
docker-compose up

# Terminal 2: Joystick bridge
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/backend
python3 joystick_bridge_host.py

# Terminal 3: Frontend
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/frontend
npm run dev
```

### Start Everything (Linux/Windows Direct):
```bash
# Terminal 1: Backend
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/backend
python3 main.py --network-monitor --network-target 192.168.1.1

# Terminal 2: Frontend
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/frontend
npm run dev
```

---

## 🎯 Expected Output

### Backend Console Should Show:
```
🚀 Mars Rover Monitoring Backend Starting...
✅ ROS Node started
🐳 Running in Docker - Expecting joystick data via HTTP from host
📡 Joystick Source: HTTP (from macOS host)
📡 Starting Network Monitor for 192.168.1.1...
✅ Network Monitor started - Monitoring 192.168.1.1
🎮 [HTTP] Joystick update received from host
📡 Network: 15.2ms latency, 0.0% loss, -45.0dBm signal
🎮 [BROADCAST] Joystick Update (HTTP (macOS Host))
```

### Joystick Bridge Should Show:
```
✅ Found 1 joystick(s)
✅ Connected: Wireless Controller
✅ Sent joystick data to backend (total: X)
```

### Frontend Should Show:
- ✅ "CONNECTED" status (green dot)
- ✅ Joystick visualization updating in real-time
- ✅ Real latency/packet loss values (not mock)
- ✅ Button status indicators

---

## 🎮 Complete Example Session

```bash
# 1. Install everything
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/backend
pip install pygame psutil requests

# 2. Start Docker backend (Terminal 1)
export USE_NETWORK_MONITOR=1
export NETWORK_TARGET_HOST=192.168.1.1
docker-compose up

# 3. Start joystick bridge (Terminal 2 - NEW TERMINAL)
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/backend
python3 joystick_bridge_host.py

# 4. Start frontend (Terminal 3 - NEW TERMINAL)
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/frontend
npm run dev

# 5. Open browser: http://localhost:5173
# 6. Connect PS4 controller and pair with Mac
# 7. Move joystick - see updates in frontend!
# 8. Check network metrics updating in real-time!
```

---

## 📝 Notes

- **macOS Docker**: Requires `joystick_bridge_host.py` to run on host
- **Linux/Windows**: Can use direct joystick (no bridge needed)
- **Network Monitor**: Works on all platforms
- **Target IP**: Use your router IP (192.168.1.1) for testing, or your rover's IP for real monitoring

Enjoy your complete setup! 🚀

