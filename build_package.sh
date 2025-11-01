#!/bin/bash
# Build the ROS2 package using Docker

echo "🔧 Building my_first_pkg with room scanner..."

docker run --rm \
  --entrypoint="" \
  -v $(pwd):/workspace \
  -w /workspace \
  picarx-ros2:humble \
  bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-select my_first_pkg"

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Build successful!"
    echo ""
    echo "Ready to run! Use:"
    echo "  ./run_room_scan.sh simulation    # Test in simulation"
    echo "  ./run_room_scan.sh                # Run with real hardware"
else
    echo "❌ Build failed!"
    exit 1
fi
