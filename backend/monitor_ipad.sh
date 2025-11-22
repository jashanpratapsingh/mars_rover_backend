#!/bin/bash
# Quick script to monitor iPad network metrics

echo "📱 Starting Network Monitor for iPad"
echo "===================================="
echo ""

# iPad IP from ARP table
IPAD_IP="169.254.13.56"
IPAD_HOSTNAME="ipad-3.local"

echo "🎯 Target Device:"
echo "   IP: $IPAD_IP"
echo "   Hostname: $IPAD_HOSTNAME"
echo ""

# Test connectivity first
echo "🔍 Testing connectivity..."
if ping -c 2 -W 2 "$IPAD_IP" > /dev/null 2>&1; then
    echo "✅ iPad is reachable at $IPAD_IP"
else
    echo "⚠️  Cannot ping iPad at $IPAD_IP"
    echo "   Trying hostname: $IPAD_HOSTNAME"
    if ping -c 2 -W 2 "$IPAD_HOSTNAME" > /dev/null 2>&1; then
        echo "✅ iPad is reachable via hostname"
        IPAD_IP="$IPAD_HOSTNAME"
    else
        echo "❌ iPad is not reachable"
        echo "   Make sure iPad is connected and on the same network"
        exit 1
    fi
fi

echo ""
echo "🚀 Starting Docker backend with network monitor..."
echo ""

# Set environment variables and start Docker
export USE_NETWORK_MONITOR=1
export NETWORK_TARGET_HOST="$IPAD_IP"
export NETWORK_PING_INTERVAL=1.0

echo "📊 Configuration:"
echo "   Target: $IPAD_IP"
echo "   Ping Interval: 1.0s"
echo ""
echo "💡 In another terminal, run:"
echo "   python3 joystick_bridge_host.py"
echo ""
echo "💡 In another terminal, start frontend:"
echo "   cd ../frontend && npm run dev"
echo ""

docker-compose up

