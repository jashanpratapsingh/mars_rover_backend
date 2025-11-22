# Docker Setup for Space Rover Backend

This Docker setup runs the backend in an Ubuntu environment, which solves all macOS pygame threading issues.

## Prerequisites

1. **Install Docker Desktop**:
   - Download from: https://www.docker.com/products/docker-desktop
   - Install and start Docker Desktop
   - Make sure Docker is running (you should see the Docker icon in your menu bar)

## Quick Start

### Option 1: Using the Script (Easiest)

```bash
cd backend
./run_docker.sh
```

### Option 2: Using Docker Compose Directly

```bash
cd backend

# Build the image
docker-compose build

# Run the container
docker-compose up
```

### Option 3: Using Docker Directly

```bash
cd backend

# Build the image
docker build -t space-rover-backend .

# Run the container
docker run -it --rm \
  -p 8765:8765 \
  --device=/dev/input/js0 \
  -v /dev/input:/dev/input:ro \
  --privileged \
  space-rover-backend
```

## Connecting Your PS4 Controller

### On macOS (Host) - Using Joystick Bridge

⚠️ **macOS Limitation**: Docker on macOS cannot directly access joystick devices.

**Solution: Run Joystick Bridge on Host**

1. **Start the Docker backend** (in one terminal):
   ```bash
   cd backend
   ./run_docker.sh
   ```

2. **In another terminal, run the joystick bridge** (on your Mac):
   ```bash
   cd backend
   pip install pygame requests  # If not already installed
   python3 joystick_bridge_host.py
   ```

   This script:
   - Reads your PS4 controller on macOS (no threading issues - runs on main thread)
   - Sends joystick data to the Docker container via HTTP
   - The Docker backend receives and processes it

3. **Connect your PS4 controller** to your Mac (Bluetooth or USB)

The joystick bridge will automatically detect and send data to the backend!

### On Linux (Host)

1. **Connect your PS4 controller**
2. **Find the device**:
   ```bash
   ls /dev/input/js*
   ```
3. **Run with Linux configuration**:
   ```bash
   docker-compose -f docker-compose.linux.yml up
   ```
   The controller will be directly accessible inside the container!

### On Linux (Host)

1. **Connect your PS4 controller**
2. **Find the device**:
   ```bash
   ls /dev/input/js*
   ```
3. **Update docker-compose.yml** if your device is not `js0`:
   ```yaml
   devices:
     - /dev/input/js1:/dev/input/js0  # Change js1 to your device
   ```

## Accessing the Backend

Once running, the backend will be available at:
- **WebSocket**: `ws://localhost:8765/ws`
- **REST API**: `http://localhost:8765/api/*`

## Viewing Logs

```bash
docker-compose logs -f
```

## Stopping the Container

Press `Ctrl+C` or run:
```bash
docker-compose down
```

## Rebuilding After Code Changes

If you change the code, rebuild:
```bash
docker-compose build --no-cache
docker-compose up
```

## Troubleshooting

### Controller Not Detected

1. **Check if controller is connected**:
   ```bash
   # On macOS
   ls /dev/input/js*
   
   # Inside container
   docker-compose exec backend ls /dev/input/js*
   ```

2. **Check container logs**:
   ```bash
   docker-compose logs backend
   ```

3. **Test joystick inside container**:
   ```bash
   docker-compose exec backend python3 test_joystick.py
   ```

### Permission Issues

If you get permission errors, you may need to:
```bash
# Add your user to the docker group (Linux)
sudo usermod -aG docker $USER
# Then logout and login again
```

### Port Already in Use

If port 8765 is already in use:
```bash
# Change the port in docker-compose.yml
ports:
  - "8766:8765"  # Use 8766 instead
```

## Benefits of Docker Approach

✅ **No macOS threading issues** - Runs in Linux (Ubuntu)  
✅ **Consistent environment** - Same setup on all machines  
✅ **Easy deployment** - Can deploy anywhere Docker runs  
✅ **Isolated** - Doesn't affect your system Python  
✅ **Reproducible** - Same environment every time  

## Development Workflow

1. **Make code changes** in your editor
2. **Rebuild container** (if needed):
   ```bash
   docker-compose build
   ```
3. **Restart container**:
   ```bash
   docker-compose restart
   ```

Or use volume mounting (already configured) - changes are reflected immediately!

