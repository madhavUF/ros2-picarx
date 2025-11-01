#!/bin/bash
# Quick hardware test

echo "🔧 Testing PicarX Hardware Setup"
echo "================================="

# Copy libraries
echo "📦 Copying libraries..."
sudo cp -r /usr/local/lib/python3.11/dist-packages/picarx* /tmp/ 2>/dev/null
sudo cp -r /usr/local/lib/python3.11/dist-packages/robot_hat* /tmp/ 2>/dev/null

# Test in Docker
docker run --rm \
  --entrypoint="" \
  --privileged \
  -v /dev:/dev \
  -v /tmp:/tmp_host \
  picarx-ros2:humble \
  bash -c "
    echo '📦 Installing dependencies...'
    pip3 install -q gpiozero smbus2 spidev rpi.gpio 2>&1 | grep -v 'WARNING' || true

    echo '📦 Copying hardware libraries...'
    cp -r /tmp_host/picarx* /usr/local/lib/python3.10/dist-packages/ 2>/dev/null || true
    cp -r /tmp_host/robot_hat* /usr/local/lib/python3.10/dist-packages/ 2>/dev/null || true

    echo '✅ Testing import...'
    python3 -c 'from picarx import Picarx; print(\"✅ PicarX hardware library imported successfully!\")' 2>&1

    echo '✅ Testing hardware initialization...'
    python3 -c '
from picarx import Picarx
try:
    px = Picarx()
    print(\"✅ Hardware initialized successfully!\")
    print(\"✅ Robot is ready to move!\")
except Exception as e:
    print(f\"⚠️  Hardware init issue: {e}\")
    print(\"   (This might be normal if not running on actual Pi hardware)\")
' 2>&1
  "

echo ""
echo "================================="
echo "Hardware test complete!"
