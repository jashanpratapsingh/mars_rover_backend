#!/usr/bin/env python3
"""
Test script to verify the joystick update endpoint is working
"""
import requests
import time

backend_url = "http://localhost:8765/api/joystick/update"

print("🧪 Testing joystick update endpoint...")
print(f"   URL: {backend_url}")
print()

# Test data
test_data = {
    "axes": [0.5, -0.3, 0.0, 0.2, 0.1, 0.0],
    "buttons": [0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    "timestamp": time.time()
}

try:
    print("📤 Sending test joystick data...")
    response = requests.post(backend_url, json=test_data, timeout=5)
    
    print(f"   Status Code: {response.status_code}")
    
    if response.status_code == 200:
        result = response.json()
        print(f"✅ Success!")
        print(f"   Response: {result}")
    else:
        print(f"❌ Error: {response.status_code}")
        print(f"   Response: {response.text}")
        
except requests.exceptions.ConnectionError:
    print("❌ Cannot connect to backend!")
    print("   Make sure Docker backend is running:")
    print("   cd backend && ./run_docker.sh")
except Exception as e:
    print(f"❌ Error: {e}")

print()
print("🧪 Testing health endpoint...")
try:
    health_response = requests.get("http://localhost:8765/api/health", timeout=2)
    if health_response.status_code == 200:
        health = health_response.json()
        print(f"✅ Backend is healthy")
        print(f"   Joystick Status: {health.get('joystick_status', 'unknown')}")
        print(f"   Joystick Source: {health.get('joystick_source', 'unknown')}")
    else:
        print(f"⚠️  Health check returned: {health_response.status_code}")
except Exception as e:
    print(f"❌ Error checking health: {e}")

