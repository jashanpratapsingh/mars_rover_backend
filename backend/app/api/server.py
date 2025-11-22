import asyncio
import json
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from typing import List, Dict, Any
import time
from app.core.config import settings
from app.ros.node import start_ros_node
from app.pipeline.processor import DataProcessor
from app.agents.anomaly import AnomalyDetectionAgent
from app.agents.prediction import PredictiveAnalyticsAgent
from app.agents.alerts import AlertManagementAgent
from app.db.database import DatabaseManager
import logging

# Configure logging
logging.basicConfig(level=settings.system.log_level)
logger = logging.getLogger(__name__)

app = FastAPI(title="Mars Rover Monitoring Backend")

# CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Global Components
ros_node = None
data_processor = DataProcessor()
anomaly_agent = AnomalyDetectionAgent()
prediction_agent = PredictiveAnalyticsAgent()
alert_agent = AlertManagementAgent()
db_manager = DatabaseManager()

# WebSocket Connection Manager
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        self.active_connections.remove(websocket)

    async def broadcast(self, message: Dict[str, Any]):
        for connection in self.active_connections:
            try:
                await connection.send_json(message)
            except Exception:
                # Handle stale connections
                pass

manager = ConnectionManager()

@app.on_event("startup")
async def startup_event():
    global ros_node
    # Start ROS Node
    ros_node, _ = start_ros_node()
    
    # Start Database
    await db_manager.init_db()
    await db_manager.start()
    
    # Start Broadcasting Loop
    asyncio.create_task(broadcast_loop())

@app.on_event("shutdown")
async def shutdown_event():
    await db_manager.stop()
    if ros_node:
        ros_node.destroy_node()

async def broadcast_loop():
    """
    Main loop: 
    1. Get data from ROS
    2. Process data
    3. Run AI agents
    4. Log to DB
    5. Broadcast to WebSockets
    """
    interval = 1.0 / settings.system.update_rate_hz
    
    while True:
        start_time = time.time()
        
        # 1. Get Data
        raw_metrics = ros_node.get_latest_metrics()
        raw_joy = ros_node.get_latest_joystick()
        
        if raw_metrics:
            # 2. Process
            processed_metrics = data_processor.process_network_metrics(raw_metrics)
            
            if processed_metrics:
                # 3. AI Agents
                anomaly_result = anomaly_agent.analyze(processed_metrics)
                prediction_result = prediction_agent.predict(processed_metrics)
                new_alert = alert_agent.process(anomaly_result, processed_metrics)
                
                # 4. Log to DB
                await db_manager.log_metrics(processed_metrics, anomaly_result)
                if new_alert:
                    await db_manager.log_alert(new_alert)
                
                # 5. Broadcast
                # Metrics
                await manager.broadcast({
                    "type": "metrics_update",
                    "payload": processed_metrics
                })
                
                # AI Insight
                ai_insight = {
                    **anomaly_result,
                    **prediction_result
                }
                await manager.broadcast({
                    "type": "ai_insight",
                    "payload": ai_insight
                })
                
                # Alert
                if new_alert:
                    await manager.broadcast({
                        "type": "alert",
                        "payload": new_alert
                    })

        if raw_joy:
            processed_joy = data_processor.process_joystick_data(raw_joy)
            if processed_joy:
                # Detect changes for logging
                # Simple logic: log if any button is pressed (value 1)
                # In a real app, you'd track state to only log on PRESS (0->1)
                # For now, we log if any button is active to demonstrate logging
                
                # We need to store previous state to detect edges, but for simplicity
                # let's just log if 'cross' (X) is pressed as an example event
                mapping = processed_joy.get('mapping', {})
                if mapping.get('cross') == 1:
                     await db_manager.log_joystick_event("button_press", {"button": "cross"})
                
                await manager.broadcast({
                    "type": "joystick_update",
                    "payload": processed_joy
                })

        # Sleep to maintain rate
        elapsed = time.time() - start_time
        await asyncio.sleep(max(0, interval - elapsed))

# --- REST Endpoints ---

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            # Keep connection alive, maybe handle incoming messages (ping/pong)
            await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)

@app.get("/api/metrics/current")
async def get_current_metrics():
    metrics = ros_node.get_latest_metrics()
    if not metrics:
        raise HTTPException(status_code=404, detail="No metrics available yet")
    return metrics

@app.get("/api/alerts/active")
async def get_active_alerts():
    return alert_agent.get_active_alerts()

@app.get("/api/health")
async def health_check():
    return {
        "status": "healthy",
        "ros_connected": ros_node is not None,
        "uptime": time.time() # Simple timestamp
    }

@app.get("/api/config")
async def get_config():
    return settings.dict()
