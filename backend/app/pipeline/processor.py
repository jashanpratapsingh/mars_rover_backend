import time
import sys
import numpy as np
from collections import deque
from typing import Dict, Any, List, Optional
from app.core.config import settings
import logging

logger = logging.getLogger(__name__)

# Platform detection for button mapping
IS_MACOS = sys.platform == "darwin"

class DataProcessor:
    def __init__(self):
        # Buffers for history
        self.metrics_buffer = deque(maxlen=settings.ai.anomaly_detection['initial_buffer_size'])
        self.joystick_buffer = deque(maxlen=100) # Keep last 100 joystick events
        
        # Validation thresholds
        self.MAX_LATENCY = 1000.0 # ms
        self.MAX_PACKET_LOSS = 100.0 # %
        self.MAX_SIGNAL = 0.0 # dBm

    def process_network_metrics(self, raw_metrics: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Validate and enrich network metrics.
        """
        if not raw_metrics:
            return None

        # Stage 1: Validation
        if not self._validate_metrics(raw_metrics):
            logger.warning(f"Invalid metrics received: {raw_metrics}")
            return None

        # Stage 2: Enrichment
        enriched_metrics = self._enrich_metrics(raw_metrics)

        # Stage 3: Buffer Management
        self.metrics_buffer.append(enriched_metrics)

        return enriched_metrics

    def process_joystick_data(self, raw_joy: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Process joystick data with PS4 Controller mapping.
        """
        if not raw_joy:
            logger.warning("process_joystick_data: raw_joy is None or empty")
            return None

        buttons = raw_joy.get('buttons', [])
        axes = raw_joy.get('axes', [])
        
        # Detect source platform for correct button mapping
        # HTTP data from macOS bridge will have _source_platform set
        source_platform = raw_joy.get('_source_platform', None)
        is_macos_source = source_platform == "darwin" or IS_MACOS
        
        logger.debug(f"Processing joystick: {len(axes)} axes, {len(buttons)} buttons (source: {source_platform or 'auto-detect'})")
        
        processed_joy = raw_joy.copy()
        processed_joy['timestamp'] = time.time()
        
        # PS4 Controller Mapping (Standard Layout)
        # Note: Mappings can vary by OS/Driver, this is a common default
        mapping = {}
        
        # Map Buttons (0-13) - handle cases with fewer buttons
        # Note: macOS pygame uses different button indices than Linux
        if len(buttons) > 0:
            if is_macos_source:
                # macOS PS4 Controller Button Mapping (pygame)
                # 0: X, 1: O, 2: Triangle, 3: Square
                # 4: Share, 5: Options, 6: PS, 7: L1, 8: R1
                # 9: L2, 10: R2, 11: L3, 12: R3
                button_mapping = {
                    "cross": buttons[0] if len(buttons) > 0 else 0,
                    "circle": buttons[1] if len(buttons) > 1 else 0,
                    "triangle": buttons[2] if len(buttons) > 2 else 0,
                    "square": buttons[3] if len(buttons) > 3 else 0,
                    "share": buttons[4] if len(buttons) > 4 else 0,
                    "options": buttons[5] if len(buttons) > 5 else 0,
                    "ps_btn": buttons[6] if len(buttons) > 6 else 0,
                    "l1": buttons[7] if len(buttons) > 7 else 0,
                    "r1": buttons[8] if len(buttons) > 8 else 0,
                    "l2_btn": buttons[9] if len(buttons) > 9 else 0,
                    "r2_btn": buttons[10] if len(buttons) > 10 else 0,
                    "l3": buttons[11] if len(buttons) > 11 else 0,
                    "r3": buttons[12] if len(buttons) > 12 else 0
                }
            else:
                # Linux/Windows PS4 Controller Button Mapping (standard)
                # 0: X, 1: O, 2: Triangle, 3: Square
                # 4: L1, 5: R1, 6: L2, 7: R2
                # 8: Share, 9: Options, 10: PS, 11: L3, 12: R3
                button_mapping = {
                    "cross": buttons[0] if len(buttons) > 0 else 0,
                    "circle": buttons[1] if len(buttons) > 1 else 0,
                    "triangle": buttons[2] if len(buttons) > 2 else 0,
                    "square": buttons[3] if len(buttons) > 3 else 0,
                    "l1": buttons[4] if len(buttons) > 4 else 0,
                    "r1": buttons[5] if len(buttons) > 5 else 0,
                    "l2_btn": buttons[6] if len(buttons) > 6 else 0,
                    "r2_btn": buttons[7] if len(buttons) > 7 else 0,
                    "share": buttons[8] if len(buttons) > 8 else 0,
                    "options": buttons[9] if len(buttons) > 9 else 0,
                    "ps_btn": buttons[10] if len(buttons) > 10 else 0,
                    "l3": buttons[11] if len(buttons) > 11 else 0,
                    "r3": buttons[12] if len(buttons) > 12 else 0
                }
            mapping.update(button_mapping)
            
        # Map Axes - handle cases with fewer axes
        # Note: PS4 controllers on macOS report Y-axis inverted, so we invert it here
        if len(axes) > 0:
            # Always map left stick if we have at least 2 axes
            if len(axes) >= 2:
                mapping["left_stick_x"] = axes[0]
                mapping["left_stick_y"] = -axes[1]  # Invert Y-axis for macOS compatibility
            
            # Map triggers if available
            if len(axes) >= 3:
                mapping["l2_axis"] = axes[2]
            
            # Map right stick if we have at least 5 axes (common layout: Lx, Ly, L2, Rx, Ry, R2)
            if len(axes) >= 4:
                mapping["right_stick_x"] = axes[3]
            if len(axes) >= 5:
                mapping["right_stick_y"] = -axes[4]  # Invert Y-axis for macOS compatibility
            if len(axes) >= 6:
                mapping["r2_axis"] = axes[5]
            
        processed_joy['mapping'] = mapping
        
        self.joystick_buffer.append(processed_joy)
        logger.debug(f"Processed joystick mapping: {mapping}")
        return processed_joy

    def _validate_metrics(self, metrics: Dict[str, Any]) -> bool:
        """
        Check if metrics are within realistic ranges.
        """
        try:
            if metrics['latency'] < 0 or metrics['latency'] > self.MAX_LATENCY:
                return False
            if metrics['packet_loss'] < 0 or metrics['packet_loss'] > self.MAX_PACKET_LOSS:
                return False
            if metrics['signal_strength'] > self.MAX_SIGNAL:
                return False
            return True
        except KeyError:
            return False

    def _enrich_metrics(self, metrics: Dict[str, Any]) -> Dict[str, Any]:
        """
        Add derived metrics and metadata.
        """
        enriched = metrics.copy()
        enriched['processed_at'] = time.time()
        
        # Calculate Jitter (variance in latency) if we have history
        if len(self.metrics_buffer) > 1:
            latencies = [m['latency'] for m in self.metrics_buffer]
            enriched['jitter'] = float(np.std(latencies))
        else:
            enriched['jitter'] = 0.0
            
        return enriched

    def get_metrics_history(self) -> List[Dict[str, Any]]:
        return list(self.metrics_buffer)
