import yaml
from pydantic import BaseModel
from pydantic_settings import BaseSettings
from typing import Dict, Any
import os

class SystemConfig(BaseModel):
    update_rate_hz: int
    log_level: str

class RosConfig(BaseModel):
    node_name: str
    topics: Dict[str, str]
    qos_history_depth: int

class AIConfig(BaseModel):
    anomaly_detection: Dict[str, Any]
    prediction: Dict[str, Any]
    alerts: Dict[str, Any]

class DatabaseConfig(BaseModel):
    url: str
    batch_insert_interval_seconds: float

class ServerConfig(BaseModel):
    host: str
    port: int

class Settings(BaseSettings):
    system: SystemConfig
    ros: RosConfig
    ai: AIConfig
    database: DatabaseConfig
    server: ServerConfig

    @classmethod
    def load(cls, config_path: str = "config.yaml") -> "Settings":
        if not os.path.exists(config_path):
            raise FileNotFoundError(f"Config file not found at {config_path}")
        
        with open(config_path, "r") as f:
            config_data = yaml.safe_load(f)
            
        return cls(**config_data)

# Global settings instance
try:
    settings = Settings.load()
except Exception as e:
    print(f"Warning: Could not load config.yaml: {e}")
    settings = None
