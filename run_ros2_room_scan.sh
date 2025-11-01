#!/bin/bash
# Run ROS2 room scanner with WORKING GPIO access for Pi 5

echo "🤖 ROS2 Room Scanner (FIXED for Pi 5)"
echo "======================================"

# Copy picarx libraries
sudo cp -r /usr/local/lib/python3.11/dist-packages/picarx* /tmp/ 2>/dev/null
sudo cp -r /usr/local/lib/python3.11/dist-packages/robot_hat* /tmp/ 2>/dev/null

# Determine mode
SIMULATION_MODE="false"
if [ "$1" = "sim" ] || [ "$1" = "simulation" ]; then
    SIMULATION_MODE="true"
    echo "📊 Mode: SIMULATION"
else
    echo "🚗 Mode: HARDWARE - Robot will actually move!"
fi

echo "🚀 Starting ROS2 with Pi 5 GPIO support..."

# Run with specific device mounting for Pi 5
docker run -it --rm \
  --entrypoint="" \
  --privileged \
  --network host \
  --device /dev/gpiomem0 \
  --device /dev/gpiomem1 \
  --device /dev/gpiomem2 \
  --device /dev/gpiomem3 \
  --device /dev/gpiomem4 \
  --device /dev/gpiochip0 \
  --device /dev/i2c-1 \
  --device /dev/i2c-10 \
  --device /dev/i2c-11 \
  -v /sys:/sys \
  -v /dev:/dev \
  -v /tmp:/tmp_host \
  -v $(pwd):/workspace \
  -w /workspace \
  picarx-ros2:humble \
  bash -c "
    echo '📦 Installing dependencies...'
    pip3 install -q gpiozero lgpio rpi-lgpio smbus2 spidev 2>&1 | grep -E '(Successfully|Requirement)' || true

    echo '📦 Installing PicarX libraries...'
    cp -r /tmp_host/picarx* /usr/local/lib/python3.10/dist-packages/ 2>/dev/null || true
    cp -r /tmp_host/robot_hat* /usr/local/lib/python3.10/dist-packages/ 2>/dev/null || true

    echo '📦 Installing ultralytics...'
    python3 -c 'import ultralytics' 2>/dev/null || {
      pip3 install -q ultralytics opencv-python 2>&1 | tail -1
    }

    echo ''
    echo '🔧 Testing hardware access...'
    python3 -c '
from picarx import Picarx
try:
    px = Picarx()
    print(\"✅ Hardware initialized - Robot can move!\")
except Exception as e:
    print(f\"⚠️  Hardware init issue: {e}\")
' 2>&1 | grep -v 'PinFactoryFallback'

    echo ''
    echo '🎯 Starting ROS2 room scanner...'
    echo ''

    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch my_first_pkg room_scan.launch.py simulation_mode:=$SIMULATION_MODE
  "

echo ""
echo "✅ ROS2 scan complete!"
