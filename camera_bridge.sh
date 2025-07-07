#!/bin/bash
# Clean camera bridge script

echo "🎥 Setting up Pi Camera Bridge"

# Load v4l2loopback
sudo modprobe v4l2loopback devices=1 video_nr=10 card_label="PiCam" exclusive_caps=1

# Check if device exists
if [ ! -e "/dev/video10" ]; then
    echo "❌ Loopback device not created"
    exit 1
fi

echo "✅ Loopback device ready: /dev/video10"

# Start the bridge (simplified)
echo "🚀 Starting camera bridge..."
libcamera-vid -t 0 --width 640 --height 480 --framerate 30 --codec yuv420 --output - --nopreview | \
ffmpeg -f rawvideo -pix_fmt yuv420p -s 640x480 -r 30 -i - -pix_fmt yuv420p -f v4l2 /dev/video10 > /dev/null 2>&1 &

BRIDGE_PID=$!
echo "✅ Bridge started (PID: $BRIDGE_PID)"

# Wait for initialization
echo "⏳ Waiting 5 seconds for bridge to initialize..."
sleep 5

# Test the bridge
echo "🧪 Testing bridge..."
python3 -c "
import cv2
import time

print('Testing loopback camera...')
cap = cv2.VideoCapture('/dev/video10', cv2.CAP_V4L2)

if cap.isOpened():
    print('Camera opened successfully')
    for i in range(3):
        ret, frame = cap.read()
        if ret and frame is not None:
            print(f'✅ Frame {i+1}: {frame.shape}')
            if i == 0:
                cv2.imwrite('camera_test.jpg', frame)
                print('📸 Saved test frame as camera_test.jpg')
            break
        else:
            print(f'Attempt {i+1}: No frame, retrying...')
            time.sleep(1)
    cap.release()
else:
    print('❌ Could not open camera')
"

echo ""
echo "🎉 Bridge test complete!"
echo "📋 To use in your code:"
echo "   cap = cv2.VideoCapture('/dev/video10', cv2.CAP_V4L2)"
echo ""
echo "🛑 Press Ctrl+C to stop the bridge"

# Keep bridge running
trap "kill $BRIDGE_PID 2>/dev/null; echo 'Bridge stopped'" EXIT
wait $BRIDGE_PID
