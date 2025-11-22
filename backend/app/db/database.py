import asyncio
import json
from datetime import datetime
from typing import List, Dict, Any, Optional
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker, declarative_base
from sqlalchemy import Column, Integer, Float, String, Boolean, DateTime, Text, JSON
from app.core.config import settings
import logging

logger = logging.getLogger(__name__)

Base = declarative_base()

# --- Models ---

class NetworkMetricsLog(Base):
    __tablename__ = "network_metrics_log"
    
    id = Column(Integer, primary_key=True, autoincrement=True)
    timestamp = Column(DateTime, index=True)
    latency = Column(Float)
    packet_loss = Column(Float)
    signal_strength = Column(Float)
    tx_rate = Column(Float)
    rx_rate = Column(Float)
    is_anomaly = Column(Boolean)
    anomaly_score = Column(Float, nullable=True)

class AlertsLog(Base):
    __tablename__ = "alerts_log"
    
    alert_id = Column(String, primary_key=True)
    type = Column(String)
    severity = Column(String)
    timestamp = Column(DateTime, index=True)
    metrics_snapshot = Column(JSON)
    confidence = Column(Float)
    recommendation = Column(Text)
    acknowledged = Column(Boolean, default=False)
    acknowledged_at = Column(DateTime, nullable=True)
    acknowledged_by = Column(String, nullable=True)

class JoystickEvents(Base):
    __tablename__ = "joystick_events"
    
    id = Column(Integer, primary_key=True, autoincrement=True)
    timestamp = Column(DateTime, index=True)
    event_type = Column(String)
    details = Column(JSON)

class SystemEvents(Base):
    __tablename__ = "system_events"
    
    id = Column(Integer, primary_key=True, autoincrement=True)
    timestamp = Column(DateTime, index=True)
    event_type = Column(String)
    message = Column(Text)
    severity = Column(String)

# --- Database Manager ---

class DatabaseManager:
    def __init__(self):
        self.use_mock = False
        try:
            self.engine = create_async_engine(settings.database.url, echo=False)
            self.async_session = sessionmaker(
                self.engine, class_=AsyncSession, expire_on_commit=False
            )
        except (ImportError, ModuleNotFoundError, Exception) as e:
            logger.warning(f"Database driver not available ({e}). Using in-memory mock.")
            self.use_mock = True
            
        self.metrics_buffer: List[NetworkMetricsLog] = []
        self.batch_interval = settings.database.batch_insert_interval_seconds
        self._running = False
        self._task = None

    async def init_db(self):
        if self.use_mock:
            return
        async with self.engine.begin() as conn:
            await conn.run_sync(Base.metadata.create_all)

    async def start(self):
        self._running = True
        self._task = asyncio.create_task(self._batch_insert_loop())

    async def stop(self):
        self._running = False
        if self._task:
            await self._task
        # Flush remaining
        await self._flush_metrics()
        if not self.use_mock:
            await self.engine.dispose()

    async def log_metrics(self, metrics: Dict[str, Any], anomaly_result: Dict[str, Any]):
        """
        Add metrics to buffer for batch insertion.
        """
        log_entry = NetworkMetricsLog(
            timestamp=datetime.fromtimestamp(metrics['timestamp']),
            latency=metrics['latency'],
            packet_loss=metrics['packet_loss'],
            signal_strength=metrics['signal_strength'],
            tx_rate=metrics['tx_rate'],
            rx_rate=metrics['rx_rate'],
            is_anomaly=anomaly_result['is_anomaly'],
            anomaly_score=anomaly_result['anomaly_score']
        )
        self.metrics_buffer.append(log_entry)

    async def log_alert(self, alert: Dict[str, Any]):
        """
        Log alert immediately (critical).
        """
        if self.use_mock:
            logger.info(f"[MOCK DB] Alert logged: {alert['type']}")
            return

        async with self.async_session() as session:
            async with session.begin():
                log_entry = AlertsLog(
                    alert_id=alert['alert_id'],
                    type=alert['type'],
                    severity=alert['severity'],
                    timestamp=datetime.fromtimestamp(alert['timestamp']),
                    metrics_snapshot=alert['metrics_snapshot'],
                    confidence=alert['confidence'],
                    recommendation=alert['recommendation'],
                    acknowledged=alert['acknowledged']
                )
                session.add(log_entry)

    async def log_system_event(self, event_type: str, message: str, severity: str = "INFO"):
        if self.use_mock:
            return

        async with self.async_session() as session:
            async with session.begin():
                log_entry = SystemEvents(
                    timestamp=datetime.now(),
                    event_type=event_type,
                    message=message,
                    severity=severity
                )
                session.add(log_entry)

    async def log_joystick_event(self, event_type: str, details: Dict[str, Any]):
        """
        Log joystick event to database.
        """
        if self.use_mock:
            return

        async with self.async_session() as session:
            async with session.begin():
                log_entry = JoystickEvents(
                    timestamp=datetime.now(),
                    event_type=event_type,
                    details=details
                )
                session.add(log_entry)

    async def _batch_insert_loop(self):
        while self._running:
            await asyncio.sleep(self.batch_interval)
            await self._flush_metrics()

    async def _flush_metrics(self):
        if not self.metrics_buffer:
            return
            
        # Swap buffer
        current_batch = self.metrics_buffer
        self.metrics_buffer = []
        
        if self.use_mock:
            # Just clear buffer in mock mode
            return

        try:
            async with self.async_session() as session:
                async with session.begin():
                    session.add_all(current_batch)
        except Exception as e:
            logger.error(f"Failed to flush metrics batch: {e}")
