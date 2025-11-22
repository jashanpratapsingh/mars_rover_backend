#!/bin/bash
# Complete startup script for Network Monitor + Joystick

set -e

echo "🚀 Starting Space Rover Backend with Network Monitor + Joystick"
echo "================================================================"
echo ""

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if we're on macOS
if [[ "$OSTYPE" == "darwin"* ]]; then
    IS_MACOS=true
    echo "🍎 Detected macOS - Will use Docker + Joystick Bridge"
else
    IS_MACOS=false
    echo "🐧 Detected Linux/Windows - Will use direct Python"
fi

# Check dependencies
echo ""
echo "📦 Checking dependencies..."

# Check Python
if ! command -v python3 &> /dev/null; then
    echo "❌ Python3 not found. Please install Python 3.8+"
    exit 1
fi
echo "✅ Python3 found"

# Check pip packages
echo "   Checking Python packages..."
python3 -c "import pygame" 2>/dev/null && echo "   ✅ pygame installed" || echo "   ⚠️  pygame not installed (run: pip install pygame)"
python3 -c "import psutil" 2>/dev/null && echo "   ✅ psutil installed" || echo "   ⚠️  psutil not installed (run: pip install psutil)"
python3 -c "import requests" 2>/dev/null && echo "   ✅ requests installed" || echo "   ⚠️  requests not installed (run: pip install requests)"

# Check Docker (for macOS)
if [ "$IS_MACOS" = true ]; then
    if ! command -v docker &> /dev/null; then
        echo "❌ Docker not found. Please install Docker Desktop for macOS"
        exit 1
    fi
    echo "✅ Docker found"
fi

# Check Node/npm (for frontend)
if ! command -v npm &> /dev/null; then
    echo "⚠️  npm not found. Frontend won't start automatically"
else
    echo "✅ npm found"
fi

echo ""
echo "================================================================"
echo ""

# Get network target (default to router)
read -p "Enter target device IP for network monitor [192.168.1.1]: " NETWORK_TARGET
NETWORK_TARGET=${NETWORK_TARGET:-192.168.1.1}

echo ""
echo "🎯 Configuration:"
echo "   Network Target: $NETWORK_TARGET"
echo "   Platform: $([ "$IS_MACOS" = true ] && echo "macOS (Docker)" || echo "Linux/Windows (Direct)")"
echo ""

# Start backend
if [ "$IS_MACOS" = true ]; then
    echo "🐳 Starting Docker backend..."
    export USE_NETWORK_MONITOR=1
    export NETWORK_TARGET_HOST=$NETWORK_TARGET
    export NETWORK_PING_INTERVAL=1.0
    
    echo ""
    echo "${YELLOW}📝 Instructions:${NC}"
    echo "   1. Docker backend is starting..."
    echo "   2. In a NEW terminal, run: ${GREEN}python3 joystick_bridge_host.py${NC}"
    echo "   3. In another terminal, run: ${GREEN}cd ../frontend && npm run dev${NC}"
    echo ""
    echo "Press Ctrl+C to stop Docker backend"
    echo ""
    
    docker-compose up
else
    echo "🐍 Starting Python backend..."
    echo ""
    echo "${YELLOW}📝 Instructions:${NC}"
    echo "   1. Backend is starting with network monitor..."
    echo "   2. In a NEW terminal, run: ${GREEN}cd ../frontend && npm run dev${NC}"
    echo ""
    echo "Press Ctrl+C to stop backend"
    echo ""
    
    python3 main.py --network-monitor --network-target "$NETWORK_TARGET"
fi

