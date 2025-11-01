#!/bin/bash
# Complete room scanning with reliable camera bridge

echo "🤖 ROOM SCANNING MISSION - HARDWARE MODE"
echo "========================================="
echo ""

# Function to check if camera bridge is working
test_camera() {
    python3 -c "
import cv2
cap = cv2.VideoCapture('/dev/video10', cv2.CAP_V4L2)
success = cap.isOpened()
cap.release()
exit(0 if success else 1)
" 2>/dev/null
    return $?
}

# Check if camera bridge is already running and working
if pgrep -x rpicam-vid > /dev/null && test_camera; then
    echo "✅ Camera bridge already running and working"
else
    echo "📷 Starting camera bridge..."

    # Kill any old bridge processes
    pkill -9 rpicam-vid 2>/dev/null
    pkill -9 ffmpeg 2>/dev/null
    sleep 1

    # Load v4l2loopback module
    if ! lsmod | grep -q v4l2loopback; then
        echo "  Loading v4l2loopback module..."
        sudo modprobe v4l2loopback devices=1 video_nr=10 card_label="PiCam" exclusive_caps=1
    fi

    if [ ! -e "/dev/video10" ]; then
        echo "❌ Failed to create loopback device"
        exit 1
    fi

    echo "  ✅ Loopback device ready: /dev/video10"

    # Start camera bridge in background
    echo "  🚀 Starting rpicam-vid..."
    rpicam-vid -t 0 --width 640 --height 480 --framerate 30 --codec yuv420 --output - --nopreview 2>/dev/null | \
    ffmpeg -f rawvideo -pix_fmt yuv420p -s 640x480 -r 30 -i - -pix_fmt yuv420p -f v4l2 /dev/video10 > /dev/null 2>&1 &

    BRIDGE_PID=$!
    echo "  ✅ Bridge started (PID: $BRIDGE_PID)"

    # Save PID for cleanup
    echo $BRIDGE_PID > /tmp/camera_bridge.pid

    # Wait for bridge to initialize
    echo "  ⏳ Waiting for camera to initialize..."
    for i in {1..10}; do
        sleep 1
        if test_camera; then
            echo "  ✅ Camera is ready!"
            break
        fi
        if [ $i -eq 10 ]; then
            echo "  ❌ Camera failed to initialize"
            exit 1
        fi
    done
fi

echo ""
echo "⚠️  IMPORTANT:"
echo "   - Robot will physically rotate 360°"
echo "   - Obstacle avoidance enabled"
echo "   - Press Ctrl+C to stop anytime"
echo ""
read -p "Ready to start? Press ENTER to continue or Ctrl+C to cancel..."
echo ""

# Run the scan
python3 room_scan_standalone.py

# Note: Don't kill the bridge - let it keep running for future scans
echo ""
echo "✅ Scan complete! Camera bridge still running for next scan."
echo "   To stop the bridge: pkill rpicam-vid"
