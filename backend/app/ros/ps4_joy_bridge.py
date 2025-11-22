"""
PS4 Controller Bridge for ROS 2
This module provides a bridge to connect PS4 controllers to ROS 2,
working with both real ROS 2 and mock environments.
"""
import threading
import time
import sys
import os
from typing import Optional, Dict, Any
import logging

logger = logging.getLogger(__name__)

try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False
    logger.warning("pygame not available. Install with: pip install pygame")


class PS4JoyBridge:
    """
    Bridge to connect PS4 controller to ROS 2 joy topic.
    Works with both real ROS 2 and mock ROS environments.
    """
    
    def __init__(self, ros_node, device_id: int = 0):
        """
        Initialize PS4 controller bridge.
        
        Args:
            ros_node: ROS node instance (real or mock)
            device_id: Joystick device ID (0 for first joystick)
        """
        self.ros_node = ros_node
        self.device_id = device_id
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.joystick: Optional[Any] = None
        self.publisher = None
        
    def start(self):
        """Start the PS4 controller bridge"""
        if not PYGAME_AVAILABLE:
            raise RuntimeError("pygame is not installed. Install with: pip install pygame")
        
        if self.running:
            logger.warning("PS4 bridge already running")
            return
        
        # Create publisher to /joy topic
        try:
            from sensor_msgs.msg import Joy
            self.publisher = self.ros_node.create_publisher(
                Joy,
                'joy',  # Standard ROS 2 joy topic
                10
            )
            logger.info("Created ROS 2 publisher for /joy topic")
        except Exception as e:
            logger.error(f"Failed to create ROS publisher: {e}")
            raise
        
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True, name="PS4JoyBridge")
        self.thread.start()
        logger.info("PS4 controller bridge started")
        
    def stop(self):
        """Stop the bridge"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.joystick:
            try:
                pygame.joystick.quit()
            except:
                pass
        logger.info("PS4 controller bridge stopped")
        
    def _read_loop(self):
        """Main loop to read PS4 controller and publish to ROS"""
        try:
            print("🔍 Initializing pygame for joystick detection...")
            # Initialize pygame (joystick doesn't require video subsystem)
            # On macOS, we need to be careful about initialization
            if sys.platform == "darwin":
                # macOS: Initialize without video to avoid main thread requirements
                # Set environment variable before init
                os.environ['SDL_VIDEODRIVER'] = 'dummy'
            pygame.init()
            pygame.joystick.init()
            
            # Get number of joysticks
            joystick_count = pygame.joystick.get_count()
            print(f"🔍 Found {joystick_count} joystick(s)")
            
            if joystick_count == 0:
                print("\n" + "="*60)
                print("❌ NO JOYSTICKS DETECTED!")
                print("="*60)
                print("\n📋 TROUBLESHOOTING STEPS:")
                print("\n1. CHECK CONNECTION:")
                print("   • Make sure your PS4 controller is connected")
                print("   • Bluetooth: Press SHARE + PS buttons, then pair")
                print("   • USB: Plug in the cable")
                
                print("\n2. CHECK SYSTEM RECOGNITION:")
                if sys.platform == "linux":
                    print("   • Run: ls /dev/input/js*")
                    print("   • If empty, controller not recognized by system")
                    print("   • Try: sudo apt install joystick")
                elif sys.platform == "darwin":  # macOS
                    print("   • Check System Preferences > Bluetooth")
                    print("   • Controller should show as 'Wireless Controller'")
                else:
                    print("   • Check Device Manager (Windows) or system settings")
                
                print("\n3. TEST PYGAME:")
                print("   • Run: python3 -c \"import pygame; pygame.init(); print('Joysticks:', pygame.joystick.get_count())\"")
                
                print("\n4. PERMISSIONS (Linux):")
                print("   • Run: sudo usermod -a -G input $USER")
                print("   • Then logout and login again")
                print("="*60 + "\n")
                
                logger.error("No joysticks found! Please connect your PS4 controller.")
                self.running = False
                return
            
            if self.device_id >= joystick_count:
                logger.warning(f"Joystick {self.device_id} not found. Using joystick 0 instead.")
                self.device_id = 0
            
            # Initialize joystick
            self.joystick = pygame.joystick.Joystick(self.device_id)
            self.joystick.init()
            
            joystick_name = self.joystick.get_name()
            num_axes = self.joystick.get_numaxes()
            num_buttons = self.joystick.get_numbuttons()
            
            print(f"✅ Connected to controller: {joystick_name}")
            print(f"   Axes: {num_axes}, Buttons: {num_buttons}")
            logger.info(f"✅ Connected to controller: {joystick_name}")
            logger.info(f"   Axes: {num_axes}, Buttons: {num_buttons}")
            
            # Test initial read to verify joystick is working
            try:
                # Try reading without event processing first
                test_axes = []
                test_buttons = []
                for i in range(min(2, num_axes)):
                    try:
                        test_axes.append(self.joystick.get_axis(i))
                    except:
                        test_axes.append(0.0)
                for i in range(min(4, num_buttons)):
                    try:
                        test_buttons.append(self.joystick.get_button(i))
                    except:
                        test_buttons.append(0)
                print(f"   Initial test read - Axes: {test_axes}, Buttons: {test_buttons}")
                
                # If we got all zeros, try with event processing
                if all(a == 0.0 for a in test_axes) and all(b == 0 for b in test_buttons):
                    print("   Note: Initial read shows zeros (joystick may be at rest)")
            except Exception as e:
                print(f"   ⚠️  Warning: Initial read test failed: {e}")
                logger.warning(f"Initial joystick read test failed: {e}")
            
            # Import Joy message type
            from sensor_msgs.msg import Joy
            
            # Read at ~30Hz (standard for joystick input)
            clock = pygame.time.Clock()
            read_attempts = 0
            empty_reads = 0
            
            while self.running:
                read_attempts += 1
                # Process pygame events to update joystick state
                # CRITICAL: On macOS, NO event functions can be called from background threads
                # Both pygame.event.pump() and pygame.event.get() require main thread
                # We must read joystick state directly without any event processing
                try:
                    if sys.platform != "darwin":
                        # Linux/Windows: Use pump() which is fine from background threads
                        pygame.event.pump()  # Process pygame events
                    # macOS: Skip all event processing - read joystick state directly
                    # The joystick state should update automatically by the OS
                except Exception as e:
                    # If event processing fails, try to continue anyway
                    # Joystick state might still be readable
                    logger.debug(f"Event processing note: {e}")
                
                # Read axes (normalize to -1.0 to 1.0 range)
                axes = []
                for i in range(num_axes):
                    try:
                        axis_value = self.joystick.get_axis(i)
                        axes.append(float(axis_value))
                    except Exception as e:
                        logger.warning(f"Error reading axis {i}: {e}")
                        axes.append(0.0)
                
                # Read buttons
                buttons = []
                for i in range(num_buttons):
                    try:
                        button_value = self.joystick.get_button(i)
                        buttons.append(int(button_value))
                    except Exception as e:
                        logger.warning(f"Error reading button {i}: {e}")
                        buttons.append(0)
                
                # Debug: Check for empty data
                if len(axes) == 0 and len(buttons) == 0:
                    empty_reads += 1
                    if empty_reads == 1 or empty_reads % 100 == 0:  # Log first occurrence and every 100th
                        if sys.platform == "darwin":
                            print(f"⚠️  Warning: Received empty joystick data (count: {empty_reads})")
                            print(f"   Note: On macOS, pygame may require event processing on main thread")
                            print(f"   Try moving the joystick - values should still update")
                        else:
                            print(f"⚠️  Warning: Received empty joystick data (count: {empty_reads})")
                        logger.warning(f"Received empty joystick data - joystick may not be updating (count: {empty_reads})")
                elif len(axes) != num_axes or len(buttons) != num_buttons:
                    logger.warning(f"Joystick data mismatch: expected {num_axes} axes, {num_buttons} buttons, got {len(axes)} axes, {len(buttons)} buttons")
                else:
                    # Reset empty reads counter if we got valid data
                    if empty_reads > 0:
                        print(f"✅ Joystick data recovered after {empty_reads} empty reads")
                        empty_reads = 0
                
                # Create Joy message
                joy_msg = Joy()
                joy_msg.axes = axes if len(axes) > 0 else [0.0] * num_axes  # Ensure we always have data
                joy_msg.buttons = buttons if len(buttons) > 0 else [0] * num_buttons  # Ensure we always have data
                
                # Publish to ROS 2 topic
                try:
                    self.publisher.publish(joy_msg)
                except Exception as e:
                    logger.error(f"Error publishing joy message: {e}")
                
                clock.tick(30)  # 30 FPS
                
        except Exception as e:
            logger.error(f"Error in PS4 controller read loop: {e}")
            import traceback
            traceback.print_exc()
            self.running = False
        finally:
            try:
                pygame.joystick.quit()
                pygame.quit()
            except:
                pass

