#!/bin/bash

# Script to run the backend in Docker (Ubuntu environment)
# This solves macOS pygame threading issues

set -e

echo "🐳 Building and running Space Rover Backend in Docker (Ubuntu)..."
echo ""

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "❌ Docker is not installed!"
    echo "   Install Docker Desktop from: https://www.docker.com/products/docker-desktop"
    exit 1
fi

# Check if docker-compose is available
if command -v docker-compose &> /dev/null; then
    COMPOSE_CMD="docker-compose"
elif docker compose version &> /dev/null; then
    COMPOSE_CMD="docker compose"
else
    echo "❌ docker-compose is not available!"
    exit 1
fi

# Detect OS and use appropriate compose file
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    echo "🐧 Detected Linux - Using Linux-specific configuration"
    COMPOSE_FILE="docker-compose.linux.yml"
else
    echo "🍎 Detected macOS - Using macOS-compatible configuration"
    COMPOSE_FILE="docker-compose.yml"
fi

# Build and run
echo "📦 Building Docker image..."
$COMPOSE_CMD -f $COMPOSE_FILE build

echo ""
echo "🚀 Starting container..."
echo "   The backend will be available at: http://localhost:8765"
echo "   Press Ctrl+C to stop"
echo ""

# Run with logs
$COMPOSE_CMD -f $COMPOSE_FILE up

