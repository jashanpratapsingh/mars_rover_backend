# Real-Time TX/RX Rate Setup

## What Was Fixed

The TX/RX rate calculation has been improved to:
- ✅ Track all active network interfaces (en0, en1, wlan0, etc.)
- ✅ Calculate real-time data transfer rates in Mbps
- ✅ Sum rates from all interfaces for total network usage
- ✅ Update every second with accurate measurements

## How It Works

1. **Network Monitor** (`network_monitor_host.py`) tracks:
   - Bytes sent/received on all network interfaces
   - Calculates rate: `(bytes_delta * 8) / (time_delta * 1_000_000)` = Mbps
   - Sends to backend every second

2. **Backend** receives and broadcasts:
   - Stores metrics in ROS node
   - Broadcasts via WebSocket every 0.1 seconds (10 Hz)
   - Frontend receives real-time updates

3. **Frontend** displays:
   - TX Rate: Outgoing data (upload)
   - RX Rate: Incoming data (download)
   - Updates in real-time

## Setup Commands

### Step 1: Install Dependencies
```bash
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/backend
pip install psutil requests
```

### Step 2: Start Everything

**Terminal 1: Docker Backend**
```bash
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/backend
docker-compose up
```

**Terminal 2: Network Monitor (iPad)**
```bash
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/backend
python3 network_monitor_host.py 169.254.13.56
```

**Terminal 3: Joystick Bridge (optional)**
```bash
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/backend
python3 joystick_bridge_host.py
```

**Terminal 4: Frontend**
```bash
cd /Users/jashanpratapsingh/personal/Projects/space_rover_ibz/frontend
npm run dev
```

## What You'll See

### Network Monitor Output:
```
✅ Sent metrics: 2.5ms latency, 0.0% loss, -45.0dBm
   TX: 1.23 Mbps, RX: 4.56 Mbps
```

### Frontend Dashboard:
- **TX Rate**: Shows real upload speed (e.g., 1.23 Mbps)
- **RX Rate**: Shows real download speed (e.g., 4.56 Mbps)
- **Updates every second** with current network activity

## Testing Real-Time Rates

To see TX/RX rates change:

1. **Download something** (browser, app update, etc.)
   - RX Rate should increase

2. **Upload something** (file upload, photo sync, etc.)
   - TX Rate should increase

3. **Stream video** (YouTube, Netflix, etc.)
   - Both TX and RX rates will show activity

4. **Idle network**
   - Rates will be low (0.0-0.1 Mbps) but not zero

## Troubleshooting

### TX/RX Still Showing 0.0

1. **Check network activity**:
   - Rates are only non-zero when data is actually transferring
   - Try downloading/uploading something to test

2. **Check network monitor logs**:
   ```bash
   # Look for TX/RX values in output
   python3 network_monitor_host.py 169.254.13.56
   ```

3. **Verify interface detection**:
   ```bash
   python3 -c "import psutil; print(list(psutil.net_io_counters(pernic=True).keys()))"
   ```

### Rates Not Updating

- Check backend is receiving: Look for `📡 [HTTP] Network metrics received`
- Check WebSocket connection: Frontend should show "CONNECTED"
- Check browser console (F12) for errors

## How Rates Are Calculated

```
TX Rate = (bytes_sent_delta * 8 bits/byte) / (time_delta * 1,000,000) = Mbps
RX Rate = (bytes_received_delta * 8 bits/byte) / (time_delta * 1,000,000) = Mbps
```

The system:
- Tracks bytes sent/received on all network interfaces
- Calculates difference over 1 second
- Converts to Mbps (Megabits per second)
- Sums all interfaces for total network usage

## Expected Values

- **Idle**: 0.0 - 0.1 Mbps
- **Web browsing**: 0.1 - 1.0 Mbps
- **Video streaming**: 1.0 - 10 Mbps
- **File download**: 5 - 100+ Mbps (depends on connection)

Enjoy real-time network rate monitoring! 📊

