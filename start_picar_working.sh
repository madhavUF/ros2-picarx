#!/bin/bash
echo "🚗 Starting PiCar-X..."

# Stop any existing container
docker stop picar-humble 2>/dev/null || true
docker rm picar-humble 2>/dev/null || true

# Start container
docker run -it \
  --privileged \
  --network host \
  -v /dev:/dev \
  -v ~/ros2_ws:/workspace \
  -w /workspace \
  --name picar-humble \
  my-picar-autonomous:latest \
  bash -c "cd /workspace && source /opt/ros/humble/setup.bash && source install/setup.bash && bash"
