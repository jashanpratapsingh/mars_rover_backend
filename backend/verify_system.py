import asyncio
import websockets
import json
import time
import requests
import subprocess
import sys
import os
import signal

async def test_websocket():
    uri = "ws://localhost:8765/ws"
    print(f"Connecting to {uri}...")
    
    received_types = set()
    
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
                    # print(f"Received: {msg_type}")
                except asyncio.TimeoutError:
                    continue
                    
    except Exception as e:
        print(f"WebSocket Error: {e}")
        return False, received_types

    print(f"Received message types: {received_types}")
    return True, received_types

def test_rest_api():
    try:
        resp = requests.get("http://localhost:8765/api/health")
        if resp.status_code == 200:
            print("Health Check: PASS")
            print(resp.json())
            return True
        else:
            print(f"Health Check Failed: {resp.status_code}")
            return False
    except Exception as e:
        print(f"REST API Error: {e}")
        return False

def main():
    # Start Mock Publisher
    print("Starting Mock Publisher...")
    pub_process = subprocess.Popen(
        [sys.executable, "app/ros/mock_publisher.py"],
        env=os.environ.copy()
    )
    
    # Start Main App
    print("Starting Main App...")
    app_process = subprocess.Popen(
        [sys.executable, "main.py"],
        env=os.environ.copy(),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    
    # Wait for startup
    time.sleep(5)
    
    try:
        # Test REST
        rest_ok = test_rest_api()
        
        # Test WebSocket
        ws_ok, msg_types = asyncio.run(test_websocket())
        
        if rest_ok and ws_ok:
            print("\n--- VERIFICATION SUCCESS ---")
            print(f"REST API: OK")
            print(f"WebSocket: OK (Types: {msg_types})")
            
            if "metrics_update" in msg_types and "joystick_update" in msg_types:
                print("Data Flow: OK")
            else:
                print("Data Flow: PARTIAL (Missing expected message types)")
                
        else:
            print("\n--- VERIFICATION FAILED ---")
            
    finally:
        print("Shutting down processes...")
        pub_process.terminate()
        app_process.terminate()
        pub_process.wait()
        app_process.wait()

if __name__ == "__main__":
    main()
