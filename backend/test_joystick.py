#!/usr/bin/env python3
"""
Quick diagnostic script to test joystick/controller detection
Run this to verify your PS4 controller is properly connected
"""
import sys
import os

print("="*60)
print("🎮 JOYSTICK DIAGNOSTIC TOOL")
print("="*60)
print()

# Check pygame
print("1. Checking pygame installation...")
try:
    import pygame
    print("   ✅ pygame is installed")
except ImportError:
    print("   ❌ pygame is NOT installed")
    print("   📦 Install with: pip install pygame")
    sys.exit(1)

# Initialize pygame
print("\n2. Initializing pygame...")
try:
    pygame.init()
    pygame.joystick.init()
    print("   ✅ pygame initialized successfully")
except Exception as e:
    print(f"   ❌ Failed to initialize pygame: {e}")
    sys.exit(1)

# Check for joysticks
print("\n3. Scanning for joysticks...")
joystick_count = pygame.joystick.get_count()
print(f"   Found {joystick_count} joystick(s)")

if joystick_count == 0:
    print("\n" + "="*60)
    print("❌ NO JOYSTICKS DETECTED!")
    print("="*60)
    print("\n📋 TROUBLESHOOTING:")
    print("\n1. CONNECTION:")
    print("   • Make sure controller is connected")
    print("   • Bluetooth: Press SHARE + PS buttons, then pair")
    print("   • USB: Plug in the cable")
    
    print("\n2. SYSTEM RECOGNITION:")
    if sys.platform == "linux":
        print("   • Check: ls /dev/input/js*")
        print("   • If empty, try: sudo apt install joystick")
        print("   • Check permissions: ls -l /dev/input/js*")
    elif sys.platform == "darwin":  # macOS
        print("   • Check: System Preferences > Bluetooth")
        print("   • Controller should show as 'Wireless Controller'")
        print("   • Try disconnecting and reconnecting")
    else:
        print("   • Check Device Manager (Windows)")
    
    print("\n3. PERMISSIONS (Linux only):")
    print("   • Run: sudo usermod -a -G input $USER")
    print("   • Then logout and login again")
    
    print("\n4. TEST AGAIN:")
    print("   • Disconnect and reconnect controller")
    print("   • Run this script again")
    print("="*60)
    sys.exit(1)

# List all joysticks
print("\n4. Joystick Details:")
for i in range(joystick_count):
    try:
        joy = pygame.joystick.Joystick(i)
        joy.init()
        print(f"\n   Joystick {i}:")
        print(f"   • Name: {joy.get_name()}")
        print(f"   • Axes: {joy.get_numaxes()}")
        print(f"   • Buttons: {joy.get_numbuttons()}")
        print(f"   • Balls: {joy.get_numballs()}")
        print(f"   • Hats: {joy.get_numhats()}")
    except Exception as e:
        print(f"   ❌ Error reading joystick {i}: {e}")

# Test input
print("\n5. Testing joystick input...")
print("   Move the joystick and press buttons!")
print("   Press Ctrl+C to exit")
print()

try:
    clock = pygame.time.Clock()
    while True:
        pygame.event.pump()
        
        for i in range(joystick_count):
            joy = pygame.joystick.Joystick(i)
            joy.init()
            
            # Read axes
            axes = [joy.get_axis(j) for j in range(joy.get_numaxes())]
            # Read buttons
            buttons = [joy.get_button(j) for j in range(joy.get_numbuttons())]
            
            # Only print if there's significant movement or button press
            if any(abs(a) > 0.1 for a in axes) or any(buttons):
                print(f"\r🎮 Joystick {i}: Axes={[f'{a:.2f}' for a in axes[:6]]} Buttons={buttons[:13]}", end="", flush=True)
        
        clock.tick(30)
        
except KeyboardInterrupt:
    print("\n\n✅ Test complete!")
    print("   If you saw joystick values changing, your controller is working!")
    print("   If not, check the troubleshooting steps above.")

finally:
    pygame.quit()

