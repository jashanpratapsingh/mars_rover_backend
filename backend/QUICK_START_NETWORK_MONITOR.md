# Quick Start: Network Monitor Commands

## Step 1: Install Dependencies

```bash
cd backend
pip install psutil
```

Or if using a virtual environment:
```bash
cd backend
python3 -m venv venv
source venv/bin/activate  # On macOS/Linux
# OR
venv\Scripts\activate  # On Windows
pip install -r requirements.txt
```

## Step 2: Find Target Device IP (Optional - for testing use router)

```bash
# Option 1: Ping common router IPs
ping 192.168.1.1
# OR
ping 192.168.0.1

# Option 2: Check your network
arp -a  # macOS/Linux
ipconfig  # Windows

# Option 3: Scan network (if nmap installed)
nmap -sn 192.168.1.0/24
```

## Step 3: Start Backend with Network Monitor

### Option A: Direct Python (Recommended)

```bash
cd backend
python3 main.py --network-monitor --network-target 192.168.1.1
```

### Option B: With Custom Ping Interval

```bash
cd backend
python3 main.py --network-monitor --network-target 192.168.1.1 --network-ping-interval 1.0
```

### Option C: Docker

```bash
cd backend
export USE_NETWORK_MONITOR=1
export NETWORK_TARGET_HOST=192.168.1.1
export NETWORK_PING_INTERVAL=1.0
docker-compose build
docker-compose up
```

## Step 4: Verify It's Working

In another terminal, check if backend is running:
```bash
curl http://localhost:8765/api/health
```

You should see network metrics in the backend console:
```
📡 Starting Network Monitor for 192.168.1.1...
✅ Network Monitor started - Monitoring 192.168.1.1
📡 Network: 15.2ms latency, 0.0% loss, -45.0dBm signal
```

## Step 5: Start Frontend (if not already running)

```bash
cd frontend
npm install  # First time only
npm run dev
```

Open browser: http://localhost:5173 (or port shown in terminal)

## Complete Command Sequence

```bash
# 1. Install dependencies
cd backend
pip install psutil

# 2. Start backend with network monitor
python3 main.py --network-monitor --network-target 192.168.1.1

# 3. In another terminal, start frontend
cd frontend
npm run dev

# 4. Open browser to see dashboard
# http://localhost:5173
```

## Troubleshooting Commands

```bash
# Test if target is reachable
ping 192.168.1.1

# Check if psutil is installed
python3 -c "import psutil; print('psutil OK')"

# Check backend is running
curl http://localhost:8765/api/health

# Check network monitor is active
# Look for "📡 Network Monitor" in backend logs
```

