import asyncio
import json
import threading
import sys
import os
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException, Request
from fastapi.middleware.cors import CORSMiddleware
from typing import List, Dict, Any
import time
from app.core.config import settings
from app.ros.node import start_ros_node
from app.pipeline.processor import DataProcessor
from app.agents.anomaly import AnomalyDetectionAgent
from app.agents.prediction import PredictiveAnalyticsAgent
from app.agents.alerts import AlertManagementAgent
from app.db.database import DatabaseManager
import logging

# Configure logging
logging.basicConfig(level=settings.system.log_level)
logger = logging.getLogger(__name__)

app = FastAPI(title="Mars Rover Monitoring Backend")

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global Components
ros_node = None
direct_joystick_reader = None  # Direct joystick reader (when not using ROS)
network_monitor_thread = None  # Network monitor thread
data_processor = DataProcessor()
anomaly_agent = AnomalyDetectionAgent()
prediction_agent = PredictiveAnalyticsAgent()
alert_agent = AlertManagementAgent()
db_manager = DatabaseManager()

# Track last joystick log time for debugging
_last_joy_log_time = 0.0

# Track joystick data source
_joystick_data_source = "NONE"  # "NONE", "ROS", "HTTP", "DIRECT"
_last_http_joystick_time = 0.0

# WebSocket Connection Manager
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    async def broadcast(self, message: Dict[str, Any]):
        for connection in self.active_connections:
            try:
                await connection.send_json(message)
            except Exception:
                # Handle stale connections
                pass

manager = ConnectionManager()

def start_ps4_joy_bridge_if_needed():
    """Start PS4 controller bridge using ROS 2 joy topic (recommended approach)"""
    global direct_joystick_reader
    use_ps4_bridge = os.environ.get("USE_PS4_BRIDGE", "1") == "1"  # Default to True
    use_direct = os.environ.get("USE_DIRECT_JOYSTICK", "0") == "1"
    
    # Check if we're in Docker - if so, expect HTTP joystick input instead
    is_docker = os.path.exists("/.dockerenv")
    if is_docker:
        print("🐳 Docker environment detected")
        print("   💡 Joystick input expected via HTTP from host (joystick_bridge_host.py)")
        print("   ⏭️  Skipping local joystick detection in container")
        # Don't try to detect joysticks in Docker - we'll get data via HTTP
        return None
    
    # If direct joystick is explicitly requested, use that instead
    if use_direct:
        return start_direct_joystick_if_needed()
    
    # Otherwise, use PS4 bridge (publishes to ROS 2 joy topic)
    if use_ps4_bridge:
        try:
            # First, verify pygame can detect joysticks
            print("🔍 Checking for joystick devices...")
            try:
                import pygame
                pygame.init()
                pygame.joystick.init()
                joystick_count = pygame.joystick.get_count()
                pygame.quit()
                
                if joystick_count == 0:
                    print("⚠️  No joysticks detected by pygame!")
                    print("   💡 If using Docker, run 'joystick_bridge_host.py' on your host")
                    print("   Attempting to use direct joystick reader as fallback...")
                    return start_direct_joystick_if_needed()
                else:
                    print(f"✅ Found {joystick_count} joystick(s) - Starting PS4 bridge...")
            except Exception as e:
                print(f"⚠️  Error checking for joysticks: {e}")
                print("   Attempting to use direct joystick reader as fallback...")
                return start_direct_joystick_if_needed()
            
            from app.ros.ps4_joy_bridge import PS4JoyBridge
            
            ps4_bridge = PS4JoyBridge(ros_node, device_id=0)
            ps4_bridge.start()
            
            # Give it a moment to initialize
            import time
            time.sleep(0.5)
            
            # Check if it's actually running
            if not ps4_bridge.running:
                print("⚠️  PS4 bridge failed to start - Falling back to direct joystick reader...")
                return start_direct_joystick_if_needed()
            
            print("✅ PS4 Controller Bridge started - Publishing to ROS 2 /joy topic")
            print("   This is the recommended approach for PS4 controllers")
            # Automatically disable mock publisher when using real joystick
            os.environ["NO_MOCK_PUBLISHER"] = "1"
            direct_joystick_reader = ps4_bridge  # Store reference
            return ps4_bridge
        except Exception as e:
            print(f"\n❌ Failed to start PS4 bridge: {e}")
            print("   Error details:")
            import traceback
            traceback.print_exc()
            print("\n   Falling back to direct joystick reader...")
            # Fall back to direct joystick reader
            return start_direct_joystick_if_needed()
    
    return None

def start_direct_joystick_if_needed():
    """Start direct joystick reader if requested (no ROS 2 needed)"""
    global direct_joystick_reader
    use_direct = os.environ.get("USE_DIRECT_JOYSTICK", "0") == "1"
    
    if use_direct:
        # Automatically disable mock publisher when using direct joystick
        os.environ["NO_MOCK_PUBLISHER"] = "1"
        
        try:
            from app.ros.direct_joystick import DirectJoystickReader
            
            def joystick_callback(joy_data: Dict[str, Any]):
                """Callback when joystick data is received"""
                # Store in ROS node's latest_joystick_data (same interface as ROS)
                global ros_node
                if ros_node:
                    with ros_node._lock:
                        ros_node.latest_joystick_data = joy_data.copy()  # Make sure we store a copy
                    
                    # Log joystick data to console (same format as ROS callback)
                    axes_list = joy_data.get('axes', [])
                    buttons_list = joy_data.get('buttons', [])
                    # Only log occasionally to avoid spam (every 0.5 seconds or when buttons pressed)
                    if any(buttons_list) or int(time.time() * 2) % 2 == 0:
                        print(f"\n🎮 [DIRECT] Joystick received - Axes: {[f'{a:.2f}' for a in axes_list]}, Buttons: {buttons_list}")
                else:
                    logger.warning("ros_node is None, cannot store joystick data")
            
            direct_joystick_reader = DirectJoystickReader(joystick_callback, device_id=0)
            direct_joystick_reader.start()
            print("✅ Direct joystick reader started - Reading from Bluetooth/USB joystick")
            print("   ⚠️  Mock publisher automatically disabled")
            return direct_joystick_reader
        except Exception as e:
            print(f"❌ Failed to start direct joystick reader: {e}")
            print("   Make sure pygame is installed: pip install pygame")
            import traceback
            traceback.print_exc()
            return None
    return None

def start_network_monitor_if_needed():
    """Start real network monitor if enabled and not in mock mode"""
    global network_monitor_thread
    
    # Check if network monitor is enabled
    use_network_monitor = os.environ.get("USE_NETWORK_MONITOR", "0") == "1"
    if not use_network_monitor:
        return False
    
    # Get target host from environment or config
    target_host = os.environ.get("NETWORK_TARGET_HOST", "192.168.1.1")
    ping_interval = float(os.environ.get("NETWORK_PING_INTERVAL", "1.0"))
    
    try:
        from app.network.publisher import start_network_publisher
        
        print(f"📡 Starting Network Monitor for {target_host}...")
        network_monitor_thread = start_network_publisher(
            target_host=target_host,
            ping_interval=ping_interval
        )
        
        # Give it a moment to initialize
        import time
        time.sleep(0.5)
        
        print(f"✅ Network Monitor started - Monitoring {target_host}")
        print(f"   Ping interval: {ping_interval}s")
        # Disable mock publisher when using real network monitor
        os.environ["NO_MOCK_PUBLISHER"] = "1"
        return True
    except Exception as e:
        print(f"❌ Failed to start network monitor: {e}")
        import traceback
        traceback.print_exc()
        return False

def start_mock_publisher_if_needed():
    """Start mock publisher if we're in mock mode (no real ROS 2) and not disabled"""
    # Check if user explicitly disabled mock publisher
    no_mock_pub = os.environ.get("NO_MOCK_PUBLISHER", "0") == "1"
    
    # Check if we're in Docker (expecting HTTP joystick input)
    is_docker = os.path.exists("/.dockerenv")
    if is_docker:
        print("⏭️  Mock publisher disabled (Docker - expecting HTTP joystick input from host)")
        return False
    
    if no_mock_pub:
        print("⏭️  Mock publisher disabled (--no-mock-publisher flag)")
        return False
    
    try:
        import rclpy
        
        # Check if rclpy module is from mocks directory
        # The mock rclpy will have __file__ pointing to backend/mocks/rclpy/__init__.py
        rclpy_module = sys.modules.get('rclpy')
        is_mock_mode = False
        
        if rclpy_module:
            # Check __file__ attribute
            module_file = getattr(rclpy_module, '__file__', '')
            module_path = str(module_file).replace('\\', '/')  # Normalize path separators
            is_mock_mode = 'mocks' in module_path or '/mock' in module_path.lower()
            
            # Also check __spec__ if available (Python 3.4+)
            if not is_mock_mode and hasattr(rclpy_module, '__spec__'):
                spec = rclpy_module.__spec__
                if spec and hasattr(spec, 'origin'):
                    origin = str(spec.origin).replace('\\', '/')
                    is_mock_mode = 'mocks' in origin or '/mock' in origin.lower()
            
            # Fallback: check if rclpy.init is the mock version (does nothing)
            # Mock rclpy.init() is just `pass`, real one might do something
            if not is_mock_mode:
                # Try to see if we can find the mock bus (mock-specific feature)
                if 'mock_ros_bus' in sys.modules:
                    is_mock_mode = True
        
        if is_mock_mode:
            print("🔧 Mock mode detected - Starting mock publisher...")
            from app.ros.mock_publisher import MockRoverPublisher
            
            def run_mock_publisher():
                try:
                    # Ensure rclpy is initialized (mock init is a no-op but safe to call)
                    rclpy.init()
                    
                    node = MockRoverPublisher()
                    print("✅ Mock Publisher node created - Generating simulated joystick and metrics data")
                    
                    # Run the mock spin loop (this will tick timers and publish data)
                    rclpy.spin(node)
                except Exception as e:
                    print(f"❌ Error in mock publisher: {e}")
                    import traceback
                    traceback.print_exc()
            
            # Start mock publisher in a daemon thread
            mock_thread = threading.Thread(target=run_mock_publisher, daemon=True, name="MockPublisher")
            mock_thread.start()
            
            # Give it a moment to start
            time.sleep(0.5)
            print("✅ Mock publisher thread started")
            return True
        else:
            print("✅ Real ROS 2 detected - Using actual joystick/rover data")
            return False
    except Exception as e:
        print(f"⚠️  Could not determine ROS mode: {e}")
        import traceback
        traceback.print_exc()
        return False

@app.on_event("startup")
async def startup_event():
    global ros_node
    print("\n" + "="*60)
    print("🚀 Mars Rover Monitoring Backend Starting...")
    print("="*60)
    
    # Start ROS Node
    ros_node, _ = start_ros_node()
    print("✅ ROS Node started and subscribed to topics")
    print(f"   - Network Metrics: {settings.ros.topics['network_metrics']}")
    print(f"   - Joystick: {settings.ros.topics['joy']}")
    
    # Check if we're in Docker
    is_docker = os.path.exists("/.dockerenv")
    
    # Declare global variable at the top before any assignments
    global direct_joystick_reader
    
    if is_docker:
        print("🐳 Running in Docker - Expecting joystick data via HTTP from host")
        print("   Run 'python3 joystick_bridge_host.py' on your Mac to send joystick data")
        direct_joystick_reader = None  # No local joystick reader in Docker
    else:
        # Check pygame availability first (only needed outside Docker)
        try:
            import pygame
            pygame_available = True
        except ImportError:
            pygame_available = False
            print("\n⚠️  WARNING: pygame is not installed!")
            print("   Install with: pip install pygame")
            print("   Without pygame, joystick input will not work.")
            print("   Continuing with mock data only...\n")
        
        # Try PS4 bridge first (recommended), then direct joystick, then mock publisher
        if pygame_available:
            direct_joystick_reader = start_ps4_joy_bridge_if_needed()
        else:
            direct_joystick_reader = None
    
    # Start network monitor if enabled (before mock publisher)
    network_monitor_started = start_network_monitor_if_needed()
    
    if not direct_joystick_reader:
        # Check if we're in Docker - if so, we expect HTTP input, don't start mock
        is_docker = os.path.exists("/.dockerenv")
        if is_docker:
            print("⏭️  Skipping mock publisher (Docker - expecting HTTP joystick input)")
            print("   💡 Run 'python3 joystick_bridge_host.py' on your Mac to send joystick data")
        else:
            # Only start mock publisher if network monitor is not active and direct joystick is not being used
            if not network_monitor_started:
                mock_started = start_mock_publisher_if_needed()
                if not mock_started:
                    print("⚠️  No joystick input source active!")
                    print("   Options:")
                    print("   1. Use --direct-joystick flag to read from Bluetooth/USB joystick")
                    print("   2. Install ROS 2 and run: ros2 run joy joy_node")
                    print("   3. Use --mock flag to generate test data")
            else:
                print("⏭️  Skipping mock publisher (real network monitor active)")
    else:
        print("🎮 Using DIRECT joystick input (real Bluetooth/USB joystick)")
    
    # Start Database
    await db_manager.init_db()
    await db_manager.start()
    print("✅ Database initialized")
    
    # Start Broadcasting Loop
    asyncio.create_task(broadcast_loop())
    print("✅ Broadcast loop started")
    print(f"✅ Server ready on http://{settings.server.host}:{settings.server.port}")
    print("="*60)
    
    # Show active joystick source
    if direct_joystick_reader:
        # Check if it's a PS4 bridge (has publisher attribute)
        try:
            from app.ros.ps4_joy_bridge import PS4JoyBridge
            if isinstance(direct_joystick_reader, PS4JoyBridge):
                print("📡 Joystick Source: PS4 BRIDGE (ROS 2 /joy topic)")
                print("   ✅ PS4 controller connected and publishing to ROS 2")
            else:
                print("📡 Joystick Source: DIRECT (Bluetooth/USB joystick)")
                print("   ✅ Real joystick input active")
        except:
            print("📡 Joystick Source: DIRECT (Bluetooth/USB joystick)")
            print("   ✅ Real joystick input active")
    else:
        use_direct = os.environ.get("USE_DIRECT_JOYSTICK", "0") == "1"
        if use_direct:
            print("📡 Joystick Source: DIRECT (but failed to start)")
            print("   ⚠️  Check if joystick is connected and pygame is installed")
        else:
            if is_docker:
                print("📡 Joystick Source: HTTP (from macOS host)")
                print("   💡 Run 'python3 joystick_bridge_host.py' on your Mac")
                print("   Waiting for joystick data from host...")
            else:
                print("📡 Joystick Source: ROS/MOCK")
                print("   Waiting for joystick data...")
    
    print("="*60 + "\n")

@app.on_event("shutdown")
async def shutdown_event():
    global direct_joystick_reader
    await db_manager.stop()
    if direct_joystick_reader:
        direct_joystick_reader.stop()
    if ros_node:
        ros_node.destroy_node()

async def broadcast_loop():
    """
    Main loop: 
    1. Get data from ROS
    2. Process data
    3. Run AI agents
    4. Log to DB
    5. Broadcast to WebSockets
    """
    global _last_joy_log_time
    interval = 1.0 / settings.system.update_rate_hz
    
    while True:
        start_time = time.time()
        
        # 1. Get Data
        raw_metrics = ros_node.get_latest_metrics()
        raw_joy = ros_node.get_latest_joystick()
        
        # Debug: Log if joystick data is available
        if raw_joy is None and direct_joystick_reader:
            # Only log occasionally to avoid spam
            current_time = time.time()
            if current_time - _last_joy_log_time > 2:
                logger.debug("Direct joystick reader active but no data received yet")
                _last_joy_log_time = current_time
        
        if raw_metrics:
            # 2. Process
            processed_metrics = data_processor.process_network_metrics(raw_metrics)
            
            if processed_metrics:
                # 3. AI Agents
                anomaly_result = anomaly_agent.analyze(processed_metrics)
                prediction_result = prediction_agent.predict(processed_metrics)
                new_alert = alert_agent.process(anomaly_result, processed_metrics)
                
                # 4. Log to DB
                await db_manager.log_metrics(processed_metrics, anomaly_result)
                if new_alert:
                    await db_manager.log_alert(new_alert)
                
                # 5. Broadcast
                # Metrics
                await manager.broadcast({
                    "type": "metrics_update",
                    "payload": processed_metrics
                })
                
                # AI Insight
                ai_insight = {
                    **anomaly_result,
                    **prediction_result
                }
                await manager.broadcast({
                    "type": "ai_insight",
                    "payload": ai_insight
                })
                
                # Alert
                if new_alert:
                    await manager.broadcast({
                        "type": "alert",
                        "payload": new_alert
                    })

        # Always process joystick data if available (even if at 0,0)
        # This ensures the frontend gets updates even when joystick is at neutral position
        # Priority: HTTP > DIRECT > ROS/MOCK
        if raw_joy:
            # Determine source for logging
            global _joystick_data_source, _last_http_joystick_time
            current_check_time = time.time()
            
            # Check if data came from HTTP recently (within last 2 seconds)
            # HTTP data takes highest priority
            is_http_data = _joystick_data_source == "HTTP" and (current_check_time - _last_http_joystick_time) < 2.0
            
            if is_http_data:
                source = "HTTP (macOS Host)"
            elif direct_joystick_reader:
                source = "DIRECT"
            else:
                source = "ROS/MOCK"
            
            processed_joy = data_processor.process_joystick_data(raw_joy)
            if processed_joy:
                mapping = processed_joy.get('mapping', {})
                axes = processed_joy.get('axes', [])
                buttons = processed_joy.get('buttons', [])
                
                # Extract stick positions
                left_x = mapping.get('left_stick_x', axes[0] if len(axes) > 0 else 0)
                left_y = mapping.get('left_stick_y', axes[1] if len(axes) > 1 else 0)
                right_x = mapping.get('right_stick_x', axes[3] if len(axes) > 3 else 0)
                right_y = mapping.get('right_stick_y', axes[4] if len(axes) > 4 else 0)
                
                # Get pressed buttons
                pressed_buttons = [k for k, v in mapping.items() if isinstance(v, (int, float)) and v == 1]
                
                # Log joystick data to console (less frequently to avoid spam)
                # Only log if there's movement or button press, or every 2 seconds
                should_log = (abs(left_x) > 0.01 or abs(left_y) > 0.01 or 
                            abs(right_x) > 0.01 or abs(right_y) > 0.01 or 
                            pressed_buttons)
                
                current_log_time = time.time()
                time_since_log = current_log_time - _last_joy_log_time
                
                if should_log or time_since_log > 2.0:
                    print(f"\n🎮 [BROADCAST] Joystick Update ({source}):")
                    print(f"   Left Stick:  X={left_x:6.2f}  Y={left_y:6.2f}")
                    print(f"   Right Stick: X={right_x:6.2f}  Y={right_y:6.2f}")
                    if pressed_buttons:
                        print(f"   Buttons: {', '.join(pressed_buttons)}")
                    print(f"   Broadcasting to {len(manager.active_connections)} WebSocket client(s)")
                    _last_joy_log_time = current_log_time
                
                # Log button presses to DB
                if pressed_buttons:
                    try:
                        await db_manager.log_joystick_event("button_press", {"buttons": pressed_buttons})
                    except Exception as e:
                        logger.error(f"Error logging joystick event: {e}")

                # Broadcast joystick update
                # Make sure payload includes both mapping and raw axes/buttons for frontend compatibility
                broadcast_payload = {
                    "axes": processed_joy.get('axes', []),
                    "buttons": processed_joy.get('buttons', []),
                    "mapping": processed_joy.get('mapping', {}),
                    "timestamp": processed_joy.get('timestamp', time.time())
                }
                
                await manager.broadcast({
                    "type": "joystick_update",
                    "payload": broadcast_payload
                })
        else:
            # Log when no joystick data is available (for debugging, every 5 seconds)
            current_time = time.time()
            if current_time - _last_joy_log_time > 5:
                logger.debug("No joystick data available from ROS node")
                _last_joy_log_time = current_time

        # Sleep to maintain rate
        elapsed = time.time() - start_time
        await asyncio.sleep(max(0, interval - elapsed))

# --- REST Endpoints ---

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    print(f"🔌 WebSocket client connected. Total clients: {len(manager.active_connections)}")
    try:
        while True:
            # Keep connection alive, maybe handle incoming messages (ping/pong)
            await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)
        print(f"🔌 WebSocket client disconnected. Remaining clients: {len(manager.active_connections)}")

@app.get("/api/metrics/current")
async def get_current_metrics():
    metrics = ros_node.get_latest_metrics()
    if not metrics:
        raise HTTPException(status_code=404, detail="No metrics available yet")
    return metrics

@app.get("/api/alerts/active")
async def get_active_alerts():
    return alert_agent.get_active_alerts()

@app.get("/api/health")
async def health_check():
    global _joystick_data_source, _last_http_joystick_time
    joystick_status = "disconnected"
    if _joystick_data_source == "HTTP":
        time_since_update = time.time() - _last_http_joystick_time
        if time_since_update < 2.0:
            joystick_status = "connected (HTTP from host)"
        else:
            joystick_status = "disconnected (no recent updates)"
    elif ros_node and ros_node.get_latest_joystick():
        joystick_status = "connected (ROS/Direct)"
    
    return {
        "status": "healthy",
        "ros_connected": ros_node is not None,
        "joystick_status": joystick_status,
        "joystick_source": _joystick_data_source,
        "uptime": time.time() # Simple timestamp
    }

@app.get("/api/config")
async def get_config():
    return settings.dict()

@app.post("/api/joystick/update")
async def receive_joystick_update(request: Request):
    """
    Receive joystick data from host (for macOS Docker setup).
    This allows the macOS host to read joystick and send to Docker container.
    """
    global _joystick_data_source, _last_http_joystick_time
    try:
        data = await request.json()
        
        axes = data.get('axes', [])
        buttons = data.get('buttons', [])
        timestamp = data.get('timestamp', time.time())
        
        # Mark data as coming from macOS (for correct button mapping)
        # Store in ROS node (same interface as ROS joystick data)
        global ros_node
        if ros_node:
            with ros_node._lock:
                ros_node.latest_joystick_data = {
                    "axes": axes,
                    "buttons": buttons,
                    "timestamp": timestamp,
                    "_source_platform": "darwin"  # macOS - for button mapping
                }
            
            _joystick_data_source = "HTTP"
            current_time = time.time()
            _last_http_joystick_time = current_time
            
            # Disable mock publisher when receiving HTTP joystick data
            # This ensures HTTP data takes priority over mock data
            os.environ["NO_MOCK_PUBLISHER"] = "1"
            
            # Log first time and occasionally (every 2 seconds)
            should_log = (_last_http_joystick_time == current_time) or (int(current_time) % 2 == 0)
            if should_log:
                print(f"\n🎮 [HTTP] Joystick update received from host - Axes: {len(axes)}, Buttons: {len(buttons)}")
                if len(axes) > 0:
                    print(f"   Sample axes: {[f'{a:.2f}' for a in axes[:4]]}")
                print(f"   ✅ Using HTTP joystick data (mock publisher disabled)")
            
            logger.debug(f"Received joystick update from host: {len(axes)} axes, {len(buttons)} buttons")
            return {"status": "ok", "received": True, "axes_count": len(axes), "buttons_count": len(buttons)}
        else:
            return {"status": "error", "message": "ROS node not initialized"}
    except Exception as e:
        logger.error(f"Error receiving joystick update: {e}")
        import traceback
        traceback.print_exc()
        return {"status": "error", "message": str(e)}
