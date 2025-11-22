import sys
import os
import threading
import time
import asyncio
import uvicorn
import requests
import websockets
import json

# Add mocks to path
sys.path.insert(0, os.path.abspath("mocks"))

# Now import app modules
from app.ros.mock_publisher import MockRoverPublisher
import rclpy
from app.api.server import app

def run_mock_publisher():
    print("Starting Mock Publisher Thread...")
    rclpy.init()
    node = MockRoverPublisher()
    try:
        # Use the mock spin which handles timers
        rclpy.spin(node)
    except Exception as e:
        print(f"Mock Publisher Error: {e}")

def run_server():
    print("Starting Server Thread...")
    uvicorn.run(app, host="0.0.0.0", port=8765, log_level="error")

async def test_websocket():
    uri = "ws://localhost:8765/ws"
    print(f"Connecting to {uri}...")
    
    received_types = set()
    
    # Give server time to start
    await asyncio.sleep(2)
    
    try:
        async with websockets.connect(uri) as websocket:
            print("Connected to WebSocket")
            
            start_time = time.time()
            while time.time() - start_time < 5:
                try:
                    message = await asyncio.wait_for(websocket.recv(), timeout=1.0)
                    data = json.loads(message)
                    msg_type = data.get("type")
                    received_types.add(msg_type)
                except asyncio.TimeoutError:
                    continue
                    
    except Exception as e:
        print(f"WebSocket Error: {e}")
        return False, received_types

    print(f"Received message types: {received_types}")
    return True, received_types

def main():
    # 1. Start Mock Publisher in Thread
    pub_thread = threading.Thread(target=run_mock_publisher, daemon=True)
    pub_thread.start()
    
    # 2. Start Server in Thread
    server_thread = threading.Thread(target=run_server, daemon=True)
    server_thread.start()
    
    # 3. Wait for startup
    time.sleep(5)
    
    # 4. Test REST
    try:
        resp = requests.get("http://localhost:8765/api/health")
        if resp.status_code == 200:
            print("Health Check: PASS")
            print(resp.json())
        else:
            print(f"Health Check Failed: {resp.status_code}")
    except Exception as e:
        print(f"REST API Error: {e}")

    # 5. Test WebSocket
    try:
        ws_ok, msg_types = asyncio.run(test_websocket())
        
        if ws_ok and "metrics_update" in msg_types:
            print("\n--- VERIFICATION SUCCESS ---")
        else:
            print("\n--- VERIFICATION FAILED ---")
            print(f"Missing metrics_update. Got: {msg_types}")
            
    except Exception as e:
        print(f"Test Error: {e}")

    # Exit
    print("Tests finished. Exiting.")
    # Force exit because threads are daemon
    os._exit(0)

if __name__ == "__main__":
    main()
