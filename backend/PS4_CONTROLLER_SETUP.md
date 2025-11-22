# PlayStation 4 Controller Setup Guide

This guide explains how to connect and use a PS4 controller with the Mars Rover Monitoring Backend.

## Quick Start

1. **Connect your PS4 controller** (see instructions below)
2. **Run the backend**:
   ```bash
   python3 main.py
   ```
   The system will automatically detect and use your PS4 controller!

## Connection Methods

### Method 1: Bluetooth (Recommended)

1. **Put controller in pairing mode**:
   - Press and hold **SHARE** + **PS** buttons simultaneously
   - The light bar will start flashing

2. **Pair with your computer**:
   - **macOS**: System Preferences > Bluetooth > Look for "Wireless Controller"
   - **Linux**: Use Bluetooth settings or `bluetoothctl`
   - **Windows**: Settings > Devices > Bluetooth

3. **Verify connection**:
   ```bash
   # On Linux
   ls /dev/input/js*
   
   # Test with jstest (Linux)
   sudo apt install joystick
   jstest /dev/input/js0
   ```

### Method 2: USB Cable

Simply connect the PS4 controller via USB cable. It should be recognized automatically.

## Installation Requirements

### Python Dependencies

```bash
pip install pygame
```

### Linux-Specific (if needed)

If your PS4 controller isn't recognized on Linux, you may need:

```bash
# Install joystick tools
sudo apt install joystick jstest-gtk

# Add user to input group (may require logout/login)
sudo usermod -a -G input $USER
```

## How It Works

The backend uses a **PS4 Joy Bridge** that:

1. Reads input from your PS4 controller using pygame
2. Publishes to the ROS 2 `/joy` topic (standard ROS 2 interface)
3. Your existing ROS node subscribes to `/joy` and processes the data
4. Data flows to the frontend via WebSocket

This approach is compatible with:
- ✅ Real ROS 2 (if installed)
- ✅ Mock ROS 2 (current setup)
- ✅ Standard ROS 2 joy_node (if you have ROS 2 installed)

## Command Line Options

```bash
# Default: Uses PS4 bridge (recommended)
python3 main.py

# Use direct joystick reader (alternative)
python3 main.py --direct-joystick

# Disable PS4 bridge, use direct reader
python3 main.py --no-ps4-bridge

# Disable mock publisher (when using real joystick)
python3 main.py --no-mock-publisher
```

## Troubleshooting

### Controller Not Detected

1. **Check connection**:
   - Verify controller is paired/connected
   - Try disconnecting and reconnecting
   - On Linux: Check `/dev/input/js*` devices

2. **Check pygame**:
   ```bash
   python3 -c "import pygame; pygame.init(); print(pygame.joystick.get_count())"
   ```
   Should show number of connected joysticks.

3. **Check permissions** (Linux):
   ```bash
   ls -l /dev/input/js*
   # If permission denied, add user to input group:
   sudo usermod -a -G input $USER
   # Then logout and login again
   ```

### No Data in Frontend

1. **Check console output**:
   - Should see: "✅ Connected to controller: [name]"
   - Should see: "🎮 [BROADCAST] Joystick Update"

2. **Check WebSocket connection**:
   - Frontend should show "CONNECTED" status
   - Check browser console (F12) for errors

3. **Verify data flow**:
   - Move joystick - you should see console output
   - Press buttons - should see button names in console

### Using Real ROS 2 (Optional)

If you have ROS 2 installed, you can use the standard `joy_node`:

```bash
# Terminal 1: Start ROS 2 joy node
ros2 run joy joy_node

# Terminal 2: Start backend
python3 main.py
```

The backend will automatically detect and use the ROS 2 joy_node data.

## PS4 Controller Button Mapping

Standard PS4 controller mapping:
- **Left Stick**: Movement (axes 0, 1)
- **Right Stick**: Camera (axes 3, 4)
- **Buttons**: 
  - 0: Cross (X)
  - 1: Circle (O)
  - 2: Triangle
  - 3: Square
  - 4: L1
  - 5: R1
  - 6: L2
  - 7: R2
  - 8: Share
  - 9: Options
  - 10: PS Button
  - 11: L3
  - 12: R3

## Additional Resources

- [ROS 2 Joy Package Documentation](https://index.ros.org/p/joy/)
- [PS4 Controller Linux Setup](https://wiki.archlinux.org/title/Gamepad)
- [pygame Joystick Documentation](https://www.pygame.org/docs/ref/joystick.html)

