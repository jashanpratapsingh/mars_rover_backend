"""
Direct joystick reader that works without ROS 2.
Reads from Bluetooth/USB joysticks directly using pygame.
"""
import threading
import time
import sys
import os
from typing import Optional, Dict, Any, Callable
import logging

logger = logging.getLogger(__name__)

try:
    import pygame
    PYGAME_AVAILABLE = True
except ImportError:
    PYGAME_AVAILABLE = False
    logger.warning("pygame not available. Install with: pip install pygame")


class DirectJoystickReader:
    """Reads joystick input directly without ROS 2"""
    
    def __init__(self, callback: Callable[[Dict[str, Any]], None], device_id: int = 0):
        """
        Initialize direct joystick reader.
        
        Args:
            callback: Function to call with joystick data: {'axes': [...], 'buttons': [...]}
            device_id: Joystick device ID (0 for first joystick)
        """
        self.callback = callback
        self.device_id = device_id
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.joystick: Optional[Any] = None
        
    def start(self):
        """Start reading joystick in background thread"""
        if not PYGAME_AVAILABLE:
            raise RuntimeError("pygame is not installed. Install with: pip install pygame")
        
        if self.running:
            logger.warning("Joystick reader already running")
            return
        
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True, name="DirectJoystickReader")
        self.thread.start()
        logger.info("Direct joystick reader started")
        
    def stop(self):
        """Stop reading joystick"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.joystick:
            try:
                pygame.joystick.quit()
            except:
                pass
        logger.info("Direct joystick reader stopped")
        
    def _read_loop(self):
        """Main loop to read joystick data"""
        try:
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
            if joystick_count == 0:
                print("\n" + "="*60)
                print("❌ NO JOYSTICKS DETECTED!")
                print("="*60)
                print("\n📋 TROUBLESHOOTING:")
                print("1. Make sure controller is connected (Bluetooth or USB)")
                print("2. For PS4: Press SHARE + PS to pair")
                print("3. Check system recognition:")
                import sys
                if sys.platform == "linux":
                    print("   Run: ls /dev/input/js*")
                elif sys.platform == "darwin":
                    print("   Check: System Preferences > Bluetooth")
                print("4. Test pygame: python3 -c \"import pygame; pygame.init(); print(pygame.joystick.get_count())\"")
                print("="*60 + "\n")
                logger.error("No joysticks found! Please connect your Bluetooth joystick.")
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
            
            logger.info(f"✅ Connected to joystick: {joystick_name}")
            logger.info(f"   Axes: {num_axes}, Buttons: {num_buttons}")
            
            # Read at ~30Hz (good for joystick input)
            clock = pygame.time.Clock()
            last_data = None
            last_update_time = 0  # Start at 0 to force initial update
            update_interval = 0.033  # ~30Hz = send update every 33ms
            
            while self.running:
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
                
                # Debug: Log if we're getting empty data
                if len(axes) == 0 and len(buttons) == 0:
                    logger.warning("Received empty joystick data - joystick may not be updating")
                elif len(axes) != num_axes or len(buttons) != num_buttons:
                    logger.warning(f"Joystick data mismatch: expected {num_axes} axes, {num_buttons} buttons, got {len(axes)} axes, {len(buttons)} buttons")
                
                # Always send updates at regular intervals (even if at 0,0) to keep frontend updated
                current_time = time.time()
                time_since_update = current_time - last_update_time
                
                # Send update if data changed OR if enough time has passed (keep frontend updated)
                current_data = (tuple(axes), tuple(buttons))
                should_update = (current_data != last_data) or (time_since_update >= update_interval)
                
                if should_update:
                    joystick_data = {
                        "axes": axes,
                        "buttons": buttons,
                        "timestamp": current_time
                    }
                    
                    try:
                        self.callback(joystick_data)
                        last_update_time = current_time
                        last_data = current_data
                    except Exception as e:
                        logger.error(f"Error in joystick callback: {e}")
                        import traceback
                        traceback.print_exc()
                
                clock.tick(30)  # 30 FPS
                
        except Exception as e:
            logger.error(f"Error in joystick read loop: {e}")
            import traceback
            traceback.print_exc()
            self.running = False
        finally:
            try:
                pygame.joystick.quit()
                pygame.quit()
            except:
                pass

