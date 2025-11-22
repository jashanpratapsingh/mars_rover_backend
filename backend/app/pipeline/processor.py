import time
import numpy as np
from collections import deque
from typing import Dict, Any, List, Optional
from app.core.config import settings
import logging

logger = logging.getLogger(__name__)

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
            return None

        processed_joy = raw_joy.copy()
        processed_joy['timestamp'] = time.time()
        
        # PS4 Controller Mapping (Standard Layout)
        # Note: Mappings can vary by OS/Driver, this is a common default
        buttons = raw_joy.get('buttons', [])
        axes = raw_joy.get('axes', [])
        
        mapping = {}
        
        # Map Buttons (0-13)
        if len(buttons) >= 13:
            mapping.update({
                "cross": buttons[0],
                "circle": buttons[1],
                "triangle": buttons[2],
                "square": buttons[3],
                "l1": buttons[4],
                "r1": buttons[5],
                "l2_btn": buttons[6],
                "r2_btn": buttons[7],
                "share": buttons[8],
                "options": buttons[9],
                "ps_btn": buttons[10],
                "l3": buttons[11],
                "r3": buttons[12]
            })
            
        # Map Axes (0-5)
        if len(axes) >= 6:
            mapping.update({
                "left_stick_x": axes[0],
                "left_stick_y": axes[1],
                "right_stick_x": axes[3], # Often 3 on Linux/ROS
                "right_stick_y": axes[4], # Often 4 on Linux/ROS
                "l2_axis": axes[2],
                "r2_axis": axes[5]
            })
            
        processed_joy['mapping'] = mapping
        
        self.joystick_buffer.append(processed_joy)
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
