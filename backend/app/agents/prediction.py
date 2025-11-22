import numpy as np
from collections import deque
from typing import Dict, Any, List
from app.core.config import settings
import logging

logger = logging.getLogger(__name__)

class PredictiveAnalyticsAgent:
    def __init__(self):
        self.history_size = settings.ai.prediction['history_buffer_size']
        self.latency_buffer = deque(maxlen=self.history_size)
        self.packet_loss_buffer = deque(maxlen=self.history_size)
        self.forecast_horizon = settings.ai.prediction['forecast_horizon_seconds'] * 10 # 10Hz * 5s = 50 samples

    def predict(self, metrics: Dict[str, Any]) -> Dict[str, Any]:
        """
        Predict future network conditions.
        """
        self.latency_buffer.append(metrics['latency'])
        self.packet_loss_buffer.append(metrics['packet_loss'])
        
        result = {
            "predicted_latency": metrics['latency'],
            "predicted_packet_loss": metrics['packet_loss'],
            "risk_level": "LOW",
            "confidence": 0,
            "time_horizon": "5 seconds"
        }
        
        if len(self.latency_buffer) < 10:
            return result
            
        # Calculate confidence based on buffer fill
        fill_ratio = len(self.latency_buffer) / self.history_size
        confidence = int(fill_ratio * 100)
        
        # Linear Regression for Latency
        pred_latency = self._forecast_metric(list(self.latency_buffer))
        pred_latency = max(0.0, pred_latency) # Cap at 0
        
        # Linear Regression for Packet Loss
        pred_loss = self._forecast_metric(list(self.packet_loss_buffer))
        pred_loss = min(max(0.0, pred_loss), 100.0) # Cap 0-100
        
        # Risk Classification
        risk = "LOW"
        if pred_latency > 150 or pred_loss > 5:
            risk = "HIGH"
        elif pred_latency > 100 or pred_loss > 2:
            risk = "MEDIUM"
            
        result.update({
            "predicted_latency": float(pred_latency),
            "predicted_packet_loss": float(pred_loss),
            "risk_level": risk,
            "confidence": confidence
        })
        
        return result

    def _forecast_metric(self, data: List[float]) -> float:
        """
        Simple linear regression forecast.
        """
        n = len(data)
        x = np.arange(n)
        y = np.array(data)
        
        # Polyfit degree 1 = linear regression
        try:
            slope, intercept = np.polyfit(x, y, 1)
            
            # Forecast
            future_x = n + self.forecast_horizon
            forecast = slope * future_x + intercept
            return forecast
        except Exception:
            return data[-1] # Fallback to last value
