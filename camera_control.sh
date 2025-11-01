#!/bin/bash
# Simple camera bridge control script

case "$1" in
    start)
        if pgrep -x rpicam-vid > /dev/null; then
            echo "✅ Camera bridge already running"
        else
            echo "🚀 Starting camera bridge..."
            ./camera_bridge.sh &
            echo "✅ Started (PID: $!)"
        fi
        ;;
    stop)
        echo "🛑 Stopping camera bridge..."
        pkill -9 rpicam-vid
        pkill -9 ffmpeg
        echo "✅ Stopped"
        ;;
    status)
        if pgrep -x rpicam-vid > /dev/null; then
            PID=$(pgrep -x rpicam-vid)
            echo "✅ Camera bridge is running (PID: $PID)"

            # Test if camera works
            if python3 -c "import cv2; cap = cv2.VideoCapture('/dev/video10', cv2.CAP_V4L2); success = cap.isOpened(); cap.release(); exit(0 if success else 1)" 2>/dev/null; then
                echo "✅ Camera is working"
            else
                echo "⚠️  Bridge running but camera not accessible"
            fi
        else
            echo "❌ Camera bridge is NOT running"
        fi
        ;;
    restart)
        echo "🔄 Restarting camera bridge..."
        $0 stop
        sleep 1
        $0 start
        ;;
    *)
        echo "Usage: $0 {start|stop|status|restart}"
        echo ""
        echo "Examples:"
        echo "  $0 start    - Start the camera bridge"
        echo "  $0 stop     - Stop the camera bridge"
        echo "  $0 status   - Check if bridge is running"
        echo "  $0 restart  - Restart the bridge"
        exit 1
        ;;
esac
