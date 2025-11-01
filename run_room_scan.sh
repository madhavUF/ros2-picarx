#!/bin/bash
# Room Scanning Mission Launcher with PROPER Hardware Support
# Usage: ./run_room_scan.sh [simulation]

echo "🤖 Room Scanning Mission"
echo "========================"

# Determine mode
SIMULATION_MODE="false"
if [ "$1" = "sim" ] || [ "$1" = "simulation" ]; then
    SIMULATION_MODE="true"
    echo "📊 Mode: SIMULATION (no hardware)"
else
    echo "🚗 Mode: HARDWARE"
    echo "⚠️  Make sure robot has 360° clear space!"
fi

# Copy picarx libraries to temp for Docker access
echo "📦 Preparing hardware libraries..."
sudo cp -r /usr/local/lib/python3.11/dist-packages/picarx* /tmp/ 2>/dev/null
sudo cp -r /usr/local/lib/python3.11/dist-packages/robot_hat* /tmp/ 2>/dev/null

echo "🚀 Launching mission..."
echo ""

# Run the mission with proper GPIO access
docker run -it --rm \
  --entrypoint="" \
  --privileged \
  --network host \
  -v /dev:/dev \
  -v /sys:/sys \
  -v /tmp:/tmp_host \
  -v $(pwd):/workspace \
  -w /workspace \
  picarx-ros2:humble \
  bash -c "
    # Install GPIO libraries that work in Docker
    echo '📦 Setting up GPIO libraries...'
    pip3 install -q lgpio gpiozero smbus2 spidev rpi.gpio 2>&1 | grep -v 'WARNING' || true

    # Copy hardware libraries
    echo '📦 Installing PicarX libraries...'
    cp -r /tmp_host/picarx* /usr/local/lib/python3.10/dist-packages/ 2>/dev/null || true
    cp -r /tmp_host/robot_hat* /usr/local/lib/python3.10/dist-packages/ 2>/dev/null || true

    # Install ultralytics for YOLO
    python3 -c 'import ultralytics' 2>/dev/null || {
      echo '📦 Installing ultralytics...'
      pip3 install -q ultralytics opencv-python 2>&1 | grep -v 'WARNING' || true
    }

    # Test hardware access
    echo '🔧 Testing hardware access...'
    python3 -c '
from picarx import Picarx
try:
    px = Picarx()
    print(\"✅ Hardware initialized - Robot ready to move!\")
except Exception as e:
    print(f\"⚠️  Hardware init warning: {e}\")
    print(\"   Will try to run anyway...\")
' 2>&1

    echo ''
    echo '🎯 Starting mission...'
    echo ''

    # Source ROS2 and run
    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch my_first_pkg room_scan.launch.py simulation_mode:=$SIMULATION_MODE
  "

echo ""
echo "✅ Mission complete!"
