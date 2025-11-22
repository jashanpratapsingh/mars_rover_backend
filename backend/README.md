# Mars Rover Monitoring Backend

A robust, production-ready backend system for a real-time Mars rover monitoring dashboard. This system integrates with ROS 2, processes telemetry using AI agents for anomaly detection and predictive analytics, and exposes a real-time API via WebSockets and REST.

## Features

- **ROS 2 Integration**: Subscribes to `network_metrics` and `joy` topics.
- **AI Agents**:
  - **Anomaly Detection**: Isolation Forest model to detect network irregularities.
  - **Predictive Analytics**: Linear regression to forecast latency and packet loss.
  - **Alert Management**: Intelligent deduplication and prioritization of alerts.
- **Real-Time API**: FastAPI-based WebSocket server broadcasting updates at 10Hz.
- **Data Persistence**: Async SQLite database for logging metrics and alerts.

## Prerequisites

- Python 3.10+
- ROS 2 Humble (or use the included mock publisher)

## Installation

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd space_rover_ibz
   ```

2. **Install dependencies**:
   
   **Option A: Using Conda (Recommended for ROS 2)**
   ```bash
   # Initialize Mamba for your shell (Run this once if 'mamba activate' fails)
   eval "$(mamba shell hook --shell zsh)"
   
   # Create environment with ROS 2 and dependencies
   mamba create -n ros_env python=3.10 -y
   mamba activate ros_env
   
   # Configure channels
   conda config --env --add channels conda-forge
   conda config --env --add channels robostack-staging
   conda config --env --remove channels defaults
   
   # Install ROS 2 and Project Deps
   mamba install ros-humble-desktop ros-humble-joy fastapi uvicorn websockets numpy pandas scikit-learn sqlalchemy aiosqlite greenlet pyyaml pydantic pydantic-settings -y
   ```

   **Option B: Using Pip (Mock Mode Only)**
   ```bash
   cd backend
   source .venv/bin/activate
   pip install -r requirements.txt
   ```

## Usage

### Running the Backend

Navigate to the backend directory and start the server:

```bash
cd backend
python3 main.py
```

### Running with Mock Data

If you don't have a real rover connected, you can run the verification script which spins up the backend along with a mock ROS 2 publisher:

```bash
python3 verify_in_process.py
```

## API Documentation

- **WebSocket**: `ws://localhost:8765/ws`
  - Receives `metrics_update`, `ai_insight`, `alert`, and `joystick_update` messages.
- **REST API**:
  - `GET /api/metrics/current`: Latest telemetry snapshot.
  - `GET /api/alerts/active`: List of active alerts.
  - `GET /api/health`: System health status.
  - `GET /api/config`: Current configuration.

## Configuration

Configuration is managed in `config.yaml`. You can adjust:
- Update rates
- ROS topic names
- AI model parameters (contamination, history buffers)
- Alert thresholds

## Project Structure

- `app/`: Main application code
  - `agents/`: AI logic (Anomaly, Prediction, Alerts)
  - `api/`: FastAPI server and routes
  - `core/`: Config and settings
  - `db/`: Database models and manager
  - `pipeline/`: Data processing and validation
  - `ros/`: ROS 2 node integration
- `mocks/`: Mock ROS 2 interfaces for testing
