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
    args = parser.parse_args()

    if args.mock:
        setup_mocks()

    uvicorn.run(
        "app.api.server:app",
        host=settings.server.host,
        port=settings.server.port,
        reload=True
    )
