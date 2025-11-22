import time
import uuid
from typing import Dict, Any, List, Optional
from app.core.config import settings

class AlertManagementAgent:
    def __init__(self):
        self.active_alerts: List[Dict[str, Any]] = []
        self.suppression_window = settings.ai.alerts['suppression_window_seconds']
        self.last_alert_times: Dict[str, float] = {}

    def process(self, anomaly_result: Dict[str, Any], metrics: Dict[str, Any]) -> Optional[Dict[str, Any]]:
        """
        Generate alerts based on anomalies and metrics.
        """
        alert_type = self._classify_anomaly(anomaly_result, metrics)
        
        if not alert_type:
            return None
            
        # Deduplication
        last_time = self.last_alert_times.get(alert_type, 0)
        if time.time() - last_time < self.suppression_window:
            return None # Suppressed
            
        # Create Alert
        alert = self._create_alert(alert_type, metrics, anomaly_result)
        
        self.active_alerts.insert(0, alert)
        self.last_alert_times[alert_type] = time.time()
        
        # Keep active alerts list manageable
        if len(self.active_alerts) > 50:
            self.active_alerts.pop()
            
        return alert

    def _classify_anomaly(self, anomaly: Dict[str, Any], metrics: Dict[str, Any]) -> Optional[str]:
        """
        Determine alert type.
        """
        if metrics['latency'] > 150:
            return "HIGH_LATENCY"
        if metrics['packet_loss'] > 5:
            return "HIGH_PACKET_LOSS"
        if metrics['signal_strength'] < -70:
            return "WEAK_SIGNAL"
        if metrics['tx_rate'] < 50 or metrics['rx_rate'] < 50:
            return "LOW_THROUGHPUT"
            
        if anomaly['is_anomaly']:
            return "GENERAL_ANOMALY"
            
        return None

    def _create_alert(self, alert_type: str, metrics: Dict[str, Any], anomaly: Dict[str, Any]) -> Dict[str, Any]:
        severity = "INFO"
        recommendation = "Monitor system."
        
        if alert_type == "HIGH_LATENCY":
            recommendation = "Network congestion. Reduce data rate."
            if metrics['latency'] > 250:
                severity = "CRITICAL"
            else:
                severity = "WARNING"
                
        elif alert_type == "HIGH_PACKET_LOSS":
            recommendation = "Interference likely. Check obstacles."
            if metrics['packet_loss'] > 10:
                severity = "CRITICAL"
            else:
                severity = "WARNING"
                
        elif alert_type == "WEAK_SIGNAL":
            recommendation = "Return to stronger signal area."
            if metrics['signal_strength'] < -80:
                severity = "CRITICAL"
            else:
                severity = "WARNING"
                
        elif alert_type == "LOW_THROUGHPUT":
            severity = "WARNING"
            recommendation = "Bandwidth saturation. Pause non-essential streams."
            
        elif alert_type == "GENERAL_ANOMALY":
            severity = "INFO"
            if anomaly['confidence'] > 70:
                severity = "WARNING"
            recommendation = "Unusual behavior detected. Prepare for manual intervention."

        return {
            "alert_id": str(uuid.uuid4()),
            "type": alert_type,
            "severity": severity,
            "timestamp": time.time(), # Use float timestamp for consistency, convert to ISO in API if needed
            "metrics_snapshot": metrics.copy(),
            "confidence": anomaly['confidence'],
            "recommendation": recommendation,
            "acknowledged": False,
            "expires_at": time.time() + 30
        }

    def get_active_alerts(self) -> List[Dict[str, Any]]:
        # Filter out expired
        current_time = time.time()
        self.active_alerts = [a for a in self.active_alerts if a['expires_at'] > current_time]
        return self.active_alerts
