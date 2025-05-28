#!/bin/bash

echo "🔍 Searching for available /dev/video* loopback device..."

# Find the first available writable v4l2loopback device
for i in {10..30}; do
    DEVICE="/dev/video$i"
    if [ -e "$DEVICE" ] && [ -w "$DEVICE" ]; then
        if v4l2-ctl --device="$DEVICE" --all 2>&1 | grep -q "LoopbackCam"; then
            LOOPBACK_DEVICE=$DEVICE
            break
        fi
    fi
done

if [ -z "$LOOPBACK_DEVICE" ]; then
    echo "❌ No available writable loopback video device found (e.g., /dev/video10)."
    echo "💡 Make sure v4l2loopback is loaded: sudo modprobe v4l2loopback"
    exit 1
fi

echo "📡 Streaming camera feed into $LOOPBACK_DEVICE ..."
libcamera-vid -t 0 --width 640 --height 480 --framerate 30 --codec yuv420 --output - \
| ffmpeg -f rawvideo -pix_fmt yuv420p -s 640x480 -r 30 -i - \
-pix_fmt yuv420p -f v4l2 "$LOOPBACK_DEVICE"
