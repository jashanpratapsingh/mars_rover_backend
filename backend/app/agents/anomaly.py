import numpy as np
from sklearn.ensemble import IsolationForest
from typing import Dict, Any, List, Optional
from app.core.config import settings
import logging

logger = logging.getLogger(__name__)

class AnomalyDetectionAgent:
    def __init__(self):
        self.model = IsolationForest(
            contamination=settings.ai.anomaly_detection['contamination'],
            random_state=42
        )
        self.buffer: List[List[float]] = []
        self.is_trained = False
        self.train_interval = settings.ai.anomaly_detection['train_interval_samples']
        self.initial_buffer_size = settings.ai.anomaly_detection['initial_buffer_size']
        self.sample_count = 0

    def analyze(self, metrics: Dict[str, Any]) -> Dict[str, Any]:
        """
        Analyze metrics for anomalies.
        """
        # Extract features: latency, packet_loss, signal_strength, tx_rate, rx_rate
        features = [
            metrics['latency'],
            metrics['packet_loss'],
            metrics['signal_strength'],
            metrics['tx_rate'],
            metrics['rx_rate']
        ]
        
        self.buffer.append(features)
        self.sample_count += 1
        
        # Check if we need to train/retrain
        if not self.is_trained:
            if len(self.buffer) >= self.initial_buffer_size:
                self._train_model()
        elif self.sample_count % self.train_interval == 0:
            self._train_model()
            
        result = {
            "is_anomaly": False,
            "anomaly_score": 0.0,
            "confidence": 0.0,
            "timestamp": metrics.get('timestamp', 0)
        }
        
        if self.is_trained:
            # Reshape for sklearn
            X = np.array([features])
            
            # Predict: 1 for normal, -1 for anomaly
            prediction = self.model.predict(X)[0]
            
            # Score: negative is anomalous
            score = self.model.decision_function(X)[0]
            
            result["is_anomaly"] = bool(prediction == -1)
            result["anomaly_score"] = float(score)
            
            # Confidence: heuristic based on score magnitude
            # If score is very negative (e.g. -0.5), high confidence it's an anomaly
            # If score is near 0, low confidence
            result["confidence"] = min(abs(score) * 200, 100.0) # Simple scaling
            
        return result

    def _train_model(self):
        """
        Train the Isolation Forest model.
        """
        if len(self.buffer) < self.initial_buffer_size:
            return

        try:
            X = np.array(self.buffer)
            # Handle NaN/Inf
            X = np.nan_to_num(X)
            
            # Check variance - if all data is same, skip training to avoid errors
            if np.all(np.var(X, axis=0) == 0):
                logger.warning("Data has zero variance, skipping model training")
                return

            self.model.fit(X)
            self.is_trained = True
            logger.info(f"Anomaly Detection Model trained on {len(self.buffer)} samples")
            
            # Keep buffer size manageable, maybe keep last N samples for next retrain
            # For now, we just keep growing it or could truncate? 
            # Requirement said "Retrain model every 1000 samples", implies using recent history.
            # Let's keep last 1000 samples max.
            if len(self.buffer) > self.train_interval:
                self.buffer = self.buffer[-self.train_interval:]
                
        except Exception as e:
            logger.error(f"Failed to train anomaly model: {e}")
