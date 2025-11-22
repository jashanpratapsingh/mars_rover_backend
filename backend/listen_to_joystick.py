import asyncio
import websockets
import json
import sys

async def listen_to_joystick():
    uri = "ws://localhost:8765/ws"
    print(f"Connecting to {uri}...")
    
    try:
        async with websockets.connect(uri) as websocket:
            print("Connected! Waiting for joystick data...")
            print("Press Ctrl+C to exit.")
            
            while True:
                try:
                    message = await websocket.recv()
                    data = json.loads(message)
                    
                    if data.get("type") == "joystick_update":
                        payload = data.get("payload", {})
                        mapping = payload.get("mapping", {})
                        
                        if mapping:
                            # Print mapped PS4 data
                            print(f"\r[PS4] L-Stick: ({mapping.get('left_stick_x', 0):.2f}, {mapping.get('left_stick_y', 0):.2f}) | "
                                  f"R-Stick: ({mapping.get('right_stick_x', 0):.2f}, {mapping.get('right_stick_y', 0):.2f}) | "
                                  f"X: {mapping.get('cross')} O: {mapping.get('circle')} "
                                  f"[]: {mapping.get('square')} ^: {mapping.get('triangle')}", end="")
                        else:
                            # Fallback to raw
                            axes = payload.get("axes", [])
                            buttons = payload.get("buttons", [])
                            axes_str = ", ".join([f"{a:.2f}" for a in axes])
                            print(f"\rJoystick: Axes=[{axes_str}] Buttons={buttons}", end="")
                            
                        sys.stdout.flush()
                        
                except websockets.exceptions.ConnectionClosed:
                    print("\nConnection closed by server.")
                    break
                    
    except Exception as e:
        print(f"\nError: {e}")
        print("Make sure the backend server is running (python3 main.py)")

if __name__ == "__main__":
    try:
        asyncio.run(listen_to_joystick())
    except KeyboardInterrupt:
        print("\nExiting...")
