#!/usr/bin/env python3
"""
Joystick Bridge for macOS Host
Reads joystick on macOS host and sends to Docker container via HTTP POST
Run this on your Mac while the backend runs in Docker

Install dependencies:
    pip install pygame requests
"""
import pygame
import requests
import time
import sys

def main():
    print("🎮 Joystick Bridge for macOS Host")
    print("=" * 60)
    print("This script reads your PS4 controller on macOS")
    print("and sends data to the backend running in Docker")
    print("=" * 60)
    print()
    
    # Initialize pygame
    pygame.init()
    pygame.joystick.init()
    
    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("❌ No joysticks found!")
        print("   Connect your PS4 controller and try again")
        sys.exit(1)
    
    print(f"✅ Found {joystick_count} joystick(s)")
    joy = pygame.joystick.Joystick(0)
    joy.init()
    
    print(f"✅ Connected: {joy.get_name()}")
    print(f"   Axes: {joy.get_numaxes()}, Buttons: {joy.get_numbuttons()}")
    print()
    print("📡 Sending data to backend at http://localhost:8765")
    print("   Make sure the Docker backend is running!")
    print("   Press Ctrl+C to stop")
    print()
    
    # Test backend connection
    print("🔍 Testing backend connection...")
    try:
        test_response = requests.get("http://localhost:8765/api/health", timeout=2)
        if test_response.status_code == 200:
            print("✅ Backend is ready!")
        else:
            print("⚠️  Backend responded but with unexpected status")
    except requests.exceptions.ConnectionError:
        print("❌ Cannot connect to backend!")
        print("   Make sure Docker backend is running:")
        print("   cd backend && ./run_docker.sh")
        sys.exit(1)
    except Exception as e:
        print(f"⚠️  Error testing backend: {e}")
    print()
    
    backend_url = "http://localhost:8765/api/joystick/update"
    
    try:
        clock = pygame.time.Clock()
        last_success_time = 0
        error_count = 0
        success_count = 0
        
        while True:
            # Process events (this is OK on main thread)
            pygame.event.pump()
            
            # Read joystick state
            axes = [joy.get_axis(i) for i in range(joy.get_numaxes())]
            buttons = [joy.get_button(i) for i in range(joy.get_numbuttons())]
            
            # Send to backend
            try:
                response = requests.post(
                    backend_url,
                    json={
                        "axes": axes,
                        "buttons": buttons,
                        "timestamp": time.time()
                    },
                    timeout=0.1
                )
                if response.status_code == 200:
                    success_count += 1
                    last_success_time = time.time()
                    error_count = 0  # Reset error count on success
                    
                    # Log first success and occasionally
                    if success_count == 1 or success_count % 300 == 0:  # Every 10 seconds at 30Hz
                        result = response.json()
                        print(f"✅ Sent joystick data to backend (total: {success_count})")
                        if result.get("received"):
                            print(f"   Backend confirmed receipt: {result.get('axes_count')} axes, {result.get('buttons_count')} buttons")
                else:
                    error_count += 1
                    if error_count == 1:
                        print(f"⚠️  Backend returned status {response.status_code}")
            except requests.exceptions.ConnectionError:
                error_count += 1
                # Backend not ready yet - will retry
                if error_count == 1 or error_count % 150 == 0:  # Log every 5 seconds
                    print("   ⏳ Cannot connect to backend - is Docker running?")
            except requests.exceptions.RequestException as e:
                error_count += 1
                if error_count == 1:
                    print(f"⚠️  Error sending data: {e}")
            
            clock.tick(30)  # 30 Hz
            
    except KeyboardInterrupt:
        print("\n✅ Stopped")
    finally:
        pygame.quit()

if __name__ == "__main__":
    main()

