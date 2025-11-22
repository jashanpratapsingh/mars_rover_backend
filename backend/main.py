import uvicorn
import sys
import os
import argparse
from app.core.config import settings

def setup_mocks():
    """Inject mocks into sys.modules if running in mock mode."""
    # Add mocks directory to path
    mocks_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "mocks"))
    if mocks_path not in sys.path:
        sys.path.insert(0, mocks_path)
    
    # We need to make sure these are importable
    # The mocks structure is mocks/rclpy etc.
    # By adding 'mocks' to path, we can import rclpy
    print(f"Mock mode enabled. Mocks path: {mocks_path}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Mars Rover Backend")
    parser.add_argument("--mock", action="store_true", help="Run with mock ROS 2 environment")
    parser.add_argument("--no-mock-publisher", action="store_true", 
                       help="Disable automatic mock publisher (use real joystick)")
    parser.add_argument("--direct-joystick", action="store_true",
                       help="Use direct joystick reading (no ROS 2, requires pygame)")
    parser.add_argument("--no-ps4-bridge", action="store_true",
                       help="Disable PS4 bridge (use direct joystick reader instead)")
    parser.add_argument("--network-monitor", action="store_true",
                       help="Enable real network monitoring (ping target device)")
    parser.add_argument("--network-target", type=str, default="192.168.1.1",
                       help="Target host IP/hostname for network monitoring (default: 192.168.1.1)")
    parser.add_argument("--network-ping-interval", type=float, default=1.0,
                       help="Ping interval in seconds (default: 1.0)")
    args = parser.parse_args()

    if args.mock:
        setup_mocks()
    
    # Store flags in environment so server can access them
    os.environ["NO_MOCK_PUBLISHER"] = "1" if args.no_mock_publisher else "0"
    os.environ["USE_DIRECT_JOYSTICK"] = "1" if args.direct_joystick else "0"
    os.environ["USE_PS4_BRIDGE"] = "0" if args.no_ps4_bridge or args.direct_joystick else "1"
    os.environ["USE_NETWORK_MONITOR"] = "1" if args.network_monitor else "0"
    os.environ["NETWORK_TARGET_HOST"] = args.network_target
    os.environ["NETWORK_PING_INTERVAL"] = str(args.network_ping_interval)

    # Disable reload in Docker/container environments
    use_reload = os.environ.get("DISABLE_RELOAD", "0") != "1" and not os.path.exists("/.dockerenv")
    
    uvicorn.run(
        "app.api.server:app",
        host=settings.server.host,
        port=settings.server.port,
        reload=use_reload
    )
