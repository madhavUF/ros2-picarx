#!/bin/bash
# Test different Docker GPIO access methods

echo "🧪 Testing Docker GPIO Access Methods"
echo "======================================"

# Method 1: --privileged (what we tried before)
echo ""
echo "Method 1: --privileged"
echo "----------------------"
docker run --rm \
  --privileged \
  -v /tmp:/tmp_host \
  picarx-ros2:humble \
  bash -c "
    pip3 install -q gpiozero smbus2 2>&1 | grep -v WARNING
    cp -r /tmp_host/picarx* /usr/local/lib/python3.10/dist-packages/ 2>/dev/null
    cp -r /tmp_host/robot_hat* /usr/local/lib/python3.10/dist-packages/ 2>/dev/null
    python3 -c 'from picarx import Picarx; px = Picarx(); print(\"✅ Method 1: SUCCESS\")' 2>&1 | tail -3
  "

# Method 2: --device /dev/gpiomem (recommended)
echo ""
echo "Method 2: --device /dev/gpiomem"
echo "--------------------------------"
docker run --rm \
  --device /dev/gpiomem \
  -v /tmp:/tmp_host \
  picarx-ros2:humble \
  bash -c "
    pip3 install -q gpiozero smbus2 2>&1 | grep -v WARNING
    cp -r /tmp_host/picarx* /usr/local/lib/python3.10/dist-packages/ 2>/dev/null
    cp -r /tmp_host/robot_hat* /usr/local/lib/python3.10/dist-packages/ 2>/dev/null
    python3 -c 'from picarx import Picarx; px = Picarx(); print(\"✅ Method 2: SUCCESS\")' 2>&1 | tail -3
  "

# Method 3: -v /sys:/sys (sysfs)
echo ""
echo "Method 3: -v /sys:/sys"
echo "----------------------"
docker run --rm \
  -v /sys:/sys \
  -v /tmp:/tmp_host \
  picarx-ros2:humble \
  bash -c "
    pip3 install -q gpiozero smbus2 2>&1 | grep -v WARNING
    cp -r /tmp_host/picarx* /usr/local/lib/python3.10/dist-packages/ 2>/dev/null
    cp -r /tmp_host/robot_hat* /usr/local/lib/python3.10/dist-packages/ 2>/dev/null
    python3 -c 'from picarx import Picarx; px = Picarx(); print(\"✅ Method 3: SUCCESS\")' 2>&1 | tail -3
  "

# Method 4: Combined approach
echo ""
echo "Method 4: --device + --privileged + /sys"
echo "-----------------------------------------"
docker run --rm \
  --privileged \
  --device /dev/gpiomem \
  -v /sys:/sys \
  -v /dev:/dev \
  -v /tmp:/tmp_host \
  picarx-ros2:humble \
  bash -c "
    pip3 install -q gpiozero smbus2 rpi.gpio lgpio 2>&1 | grep -v WARNING
    cp -r /tmp_host/picarx* /usr/local/lib/python3.10/dist-packages/ 2>/dev/null
    cp -r /tmp_host/robot_hat* /usr/local/lib/python3.10/dist-packages/ 2>/dev/null
    python3 -c 'from picarx import Picarx; px = Picarx(); print(\"✅ Method 4: SUCCESS\")' 2>&1 | tail -3
  "

echo ""
echo "======================================"
echo "Test Complete!"
