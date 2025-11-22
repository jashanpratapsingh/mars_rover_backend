# HTTP Joystick Setup for Docker (macOS)

## Overview

When running the backend in Docker on macOS, the container cannot directly access USB/Bluetooth joysticks. This setup uses an HTTP bridge where:

1. **macOS Host**: Runs `joystick_bridge_host.py` to read joystick input
2. **Docker Container**: Receives joystick data via HTTP POST endpoint
3. **Frontend**: Receives real-time updates via WebSocket

## How It Works

### Data Flow
```
PS4 Controller (Bluetooth) 
    ↓
macOS Host (joystick_bridge_host.py)
    ↓ HTTP POST /api/joystick/update
Docker Container (backend)
    ↓ WebSocket
Frontend (React)
```

### Priority System
The backend prioritizes joystick data sources in this order:
1. **HTTP** (from macOS host) - Highest priority
2. **DIRECT** (local pygame reader)
3. **ROS/MOCK** (ROS 2 topics or mock data) - Lowest priority

When HTTP data is received:
- Mock publisher is automatically disabled
- HTTP data overwrites any existing joystick data
- Frontend receives real-time updates via WebSocket

## Setup Instructions

### Step 1: Start Docker Backend
```bash
cd backend
./run_docker.sh
```

You should see:
```
🐳 Running in Docker - Expecting joystick data via HTTP from host
📡 Joystick Source: HTTP (from macOS host)
   💡 Run 'python3 joystick_bridge_host.py' on your Mac
   Waiting for joystick data from host...
```

### Step 2: Start Joystick Bridge (on macOS Host)
In a **new terminal** (on your Mac, not in Docker):
```bash
cd backend
python3 joystick_bridge_host.py
```

You should see:
```
✅ Backend is reachable at http://localhost:8765
🎮 Starting joystick bridge...
✅ Found 1 joystick(s)
✅ Sent joystick data to backend (success count: X)
```

### Step 3: Verify Frontend Connection
1. Open the frontend in your browser
2. Check the connection status (should show "CONNECTED")
3. Move the joystick - you should see real-time updates in the joystick visualization

## Testing

### Test the HTTP Endpoint
```bash
cd backend
python3 test_joystick_endpoint.py
```

This will:
- Test if the endpoint is accessible
- Send test joystick data
- Show the joystick status from the health endpoint

### Check Docker Logs
```bash
docker-compose logs -f backend
```

Look for:
- `🎮 [HTTP] Joystick update received from host`
- `🎮 [BROADCAST] Joystick Update (HTTP (macOS Host))`
- `Broadcasting to X WebSocket client(s)`

### Check Frontend Console
Open browser DevTools (F12) and look for:
- `✅ Connected to Backend WebSocket`
- `🎮 Joystick update received:`
- `🎮 Processed joystick data:`

## Troubleshooting

### "No joysticks detected by pygame!" in Docker
✅ **This is expected!** Docker containers cannot access host hardware. The warning is now suppressed when running in Docker.

### Frontend not updating
1. Check WebSocket connection status in frontend
2. Verify `joystick_bridge_host.py` is running and sending data
3. Check Docker logs for `[BROADCAST]` messages
4. Verify frontend console shows `🎮 Joystick update received`

### Mock data still showing
- HTTP data automatically disables mock publisher
- If you see mock data, check that `joystick_bridge_host.py` is running
- Verify HTTP endpoint is receiving data: `python3 test_joystick_endpoint.py`

### Backend not receiving HTTP data
1. Check backend is running: `curl http://localhost:8765/api/health`
2. Verify bridge script can reach backend: Check for "✅ Backend is reachable"
3. Check network: Ensure Docker port 8765 is mapped correctly

## Architecture Notes

- **HTTP Endpoint**: `/api/joystick/update` (POST)
- **WebSocket**: `ws://localhost:8765/ws`
- **Data Format**: JSON with `axes`, `buttons`, and `timestamp`
- **Update Rate**: ~20Hz (50ms intervals) from bridge script
- **Priority**: HTTP data always takes precedence over mock/ROS data

