#!/bin/bash
# Run room scan natively on the Pi (no Docker)
# This allows direct hardware access

echo "🤖 Room Scanning Mission (Native Mode)"
echo "========================================"

# Check if we're on the Pi
if [ ! -d "/sys/class/gpio" ]; then
    echo "❌ Error: GPIO not found. Are you running on a Raspberry Pi?"
    exit 1
fi

# Determine mode
SIMULATION_MODE="false"
if [ "$1" = "sim" ] || [ "$1" = "simulation" ]; then
    SIMULATION_MODE="true"
    echo "📊 Mode: SIMULATION (no hardware)"
else
    echo "🚗 Mode: HARDWARE - Robot will physically move!"
fi

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ Error: ROS2 not found!"
    echo "   This system doesn't have ROS2 installed directly."
    echo "   You must use Docker: ./run_room_scan.sh"
    exit 1
fi

echo "🚀 Launching mission..."
echo ""

# Source workspace
cd $(dirname $0)
source install/setup.bash

# Run the launch file
ros2 launch my_first_pkg room_scan.launch.py simulation_mode:=$SIMULATION_MODE

echo ""
echo "✅ Mission complete!"
