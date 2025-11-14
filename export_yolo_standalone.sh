#!/bin/bash
#
# Export Standalone YOLO Detector Package
# No ROS dependencies - pure Python
#

echo "========================================="
echo "Exporting Standalone YOLO Detector"
echo "========================================="

# Create export directory
EXPORT_DIR="yolo_standalone_export_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$EXPORT_DIR"

echo "Export directory: $EXPORT_DIR"

# Copy YOLO model
echo "Copying YOLO model file..."
cp yolov8n.pt "$EXPORT_DIR/"

# Copy standalone script
echo "Copying standalone detector script..."
cp yolo_detector_standalone.py "$EXPORT_DIR/"

# Create requirements file
echo "Creating requirements.txt..."
cat > "$EXPORT_DIR/requirements.txt" << EOF
# YOLO and Vision Dependencies
ultralytics>=8.0.0
opencv-python>=4.8.0
torch>=2.0.0
torchvision>=0.15.0
numpy>=1.24.0
EOF

# Create installation script
echo "Creating install script..."
cat > "$EXPORT_DIR/install.sh" << 'EOF'
#!/bin/bash
# Installation script for Standalone YOLO Detector

echo "========================================="
echo "Installing Standalone YOLO Detector"
echo "========================================="

# Check Python version
PYTHON_VERSION=$(python3 --version 2>&1 | awk '{print $2}')
echo "Detected Python version: $PYTHON_VERSION"

# Install Python dependencies
echo ""
echo "Installing Python dependencies..."
pip3 install -r requirements.txt

if [ $? -eq 0 ]; then
    echo ""
    echo "========================================="
    echo "Installation complete!"
    echo "========================================="
    echo ""
    echo "Quick Start:"
    echo "  # Basic usage"
    echo "  python3 yolo_detector_standalone.py"
    echo ""
    echo "  # Show live video"
    echo "  python3 yolo_detector_standalone.py --display"
    echo ""
    echo "  # Use specific camera"
    echo "  python3 yolo_detector_standalone.py --camera /dev/video0"
    echo ""
    echo "  # Save annotated video"
    echo "  python3 yolo_detector_standalone.py --save-video output.mp4"
    echo ""
    echo "  # Run for 60 seconds"
    echo "  python3 yolo_detector_standalone.py --duration 60"
    echo ""
    echo "For more options:"
    echo "  python3 yolo_detector_standalone.py --help"
    echo ""
else
    echo ""
    echo "========================================="
    echo "Installation failed!"
    echo "========================================="
    echo "Please check the error messages above"
    exit 1
fi
EOF

chmod +x "$EXPORT_DIR/install.sh"

# Create README
cat > "$EXPORT_DIR/README.md" << 'EOF'
# Standalone YOLO Object Detector

Simple, standalone YOLO object detection with **no ROS dependencies**. Just Python, OpenCV, and YOLO.

## Features

- ✅ **No ROS required** - Pure Python
- ✅ **Easy to use** - Single script with command-line options
- ✅ **Flexible** - Works with any USB camera
- ✅ **Real-time detection** - 10 FPS default (configurable)
- ✅ **80 object classes** - Pre-trained COCO dataset
- ✅ **Optional video display** - See detections live
- ✅ **Video recording** - Save annotated videos
- ✅ **Detection statistics** - Summary of detected objects

## Contents

- `yolo_detector_standalone.py` - Main detection script
- `yolov8n.pt` - YOLOv8 nano model (6.2MB)
- `requirements.txt` - Python dependencies
- `install.sh` - Automated installation script
- `README.md` - This file

## Requirements

- **Python**: 3.6+ (3.8+ recommended)
- **Camera**: USB camera or compatible video device
- **Optional**: CUDA for GPU acceleration
- **No ROS required!**

## Quick Installation

### Automated Installation

```bash
./install.sh
```

### Manual Installation

```bash
pip3 install -r requirements.txt
```

## Usage

### Basic Examples

```bash
# Basic detection (console output only)
python3 yolo_detector_standalone.py

# Show live video with detection boxes
python3 yolo_detector_standalone.py --display

# Use specific camera device
python3 yolo_detector_standalone.py --camera /dev/video0

# Save annotated video
python3 yolo_detector_standalone.py --save-video detections.mp4

# Run for specific duration
python3 yolo_detector_standalone.py --duration 60

# Higher confidence threshold
python3 yolo_detector_standalone.py --confidence 0.7

# Combined options
python3 yolo_detector_standalone.py --display --save-video out.mp4 --fps 15
```

### All Command-Line Options

```
--camera DEVICE       Camera device (default: 0 for /dev/video0)
                      Can be int (0, 1, 2) or path (/dev/video0)

--model PATH          Path to YOLO model (default: yolov8n.pt)

--confidence FLOAT    Detection confidence threshold 0-1 (default: 0.5)
                      Higher = fewer but more accurate detections

--display             Show video window with detection boxes

--save-video PATH     Save annotated video to file (e.g., output.mp4)

--fps INT             Target frames per second (default: 10)
                      Lower = less CPU, Higher = smoother video

--duration SECONDS    Run for specified seconds (default: infinite)

--max-frames INT      Process max number of frames (default: infinite)

--help                Show help message
```

## Example Output

```
Loading YOLO model: yolov8n.pt
YOLO model loaded successfully
Opening camera: 0
Camera opened: 640x480

Starting continuous detection...
Press Ctrl+C to stop

Frame 10: 2 detections - person (0.92), laptop (0.78)
Frame 30: 1 detections - person (0.88)
Frame 50: 3 detections - person (0.91), cell phone (0.65), cup (0.54)

^C
Stopped by user

============================================================
DETECTION SUMMARY
============================================================
Duration: 45.2 seconds
Total Frames: 452
Average FPS: 10.0

Object Detections:
  person: 340
  laptop: 120
  cell phone: 45
  cup: 23
============================================================
```

## Camera Configuration

### Find Available Cameras

```bash
# Linux
ls -l /dev/video*

# Test camera
v4l2-ctl --list-devices
```

### Common Camera Devices

- `/dev/video0` - First USB camera (use `--camera 0`)
- `/dev/video1` - Second USB camera (use `--camera 1`)
- `/dev/video10` - May be used by rpicam-vid on Raspberry Pi

## Integration with Your Code

### As a Python Module

```python
from yolo_detector_standalone import YOLODetector

# Create detector
detector = YOLODetector(
    model_path='yolov8n.pt',
    camera_device=0,
    confidence=0.5
)

# Process single frame
success, detections = detector.detect_frame()

for det in detections:
    print(f"Detected: {det['class']} with confidence {det['confidence']:.2f}")
    print(f"  Bounding box: {det['bbox']}")
    print(f"  Center: {det['center']}")
    print(f"  Area: {det['area']:.0f} pixels")

# Cleanup
detector.cleanup()
```

### Process N Frames

```python
detector = YOLODetector(model_path='yolov8n.pt')

for i in range(100):  # Process 100 frames
    success, detections = detector.detect_frame()
    if not success:
        break
    # Your processing here...

detector.print_summary()
detector.cleanup()
```

## Troubleshooting

### Camera Not Found

```bash
# Check available cameras
ls -l /dev/video*

# Try different camera index
python3 yolo_detector_standalone.py --camera 1
python3 yolo_detector_standalone.py --camera /dev/video10
```

### Permission Denied (Camera)

```bash
# Add user to video group
sudo usermod -a -G video $USER

# Logout and login again
```

### Python Module Not Found

```bash
# Reinstall dependencies
pip3 install -r requirements.txt

# Or install individually
pip3 install ultralytics opencv-python torch
```

### Low FPS / Performance Issues

```bash
# Reduce FPS target
python3 yolo_detector_standalone.py --fps 5

# Increase confidence threshold (fewer detections)
python3 yolo_detector_standalone.py --confidence 0.7

# Use GPU (if CUDA available)
# PyTorch with CUDA will automatically use GPU
```

### Display Not Working

If `--display` doesn't work (headless server, SSH session):
- Remove `--display` flag
- Use `--save-video` instead to record video
- View detection output in console

## Object Classes

The YOLOv8n model detects 80 COCO classes:

**People & Animals:**
person, cat, dog, horse, sheep, cow, elephant, bear, zebra, giraffe

**Vehicles:**
bicycle, car, motorcycle, airplane, bus, train, truck, boat

**Indoor Objects:**
bottle, wine glass, cup, fork, knife, spoon, bowl, banana, apple, sandwich,
orange, broccoli, carrot, hot dog, pizza, donut, cake, chair, couch,
potted plant, bed, dining table, toilet, tv, laptop, mouse, remote, keyboard,
cell phone, microwave, oven, toaster, sink, refrigerator, book, clock, vase

**Outdoor & Sports:**
traffic light, fire hydrant, stop sign, parking meter, bench, bird,
backpack, umbrella, handbag, tie, suitcase, frisbee, skis, snowboard,
sports ball, kite, baseball bat, baseball glove, skateboard, surfboard,
tennis racket, scissors, teddy bear, hair drier, toothbrush

## Custom YOLO Models

To use your own trained model:

```bash
python3 yolo_detector_standalone.py --model /path/to/your/model.pt
```

## Performance Tips

1. **Lower FPS** for CPU-limited systems: `--fps 5`
2. **Higher confidence** for cleaner output: `--confidence 0.7`
3. **GPU acceleration**: Install PyTorch with CUDA
4. **Smaller resolution**: Modify camera settings or resize frames

## License

MIT License

## Support

For issues or questions:
- YOLOv8 Documentation: https://docs.ultralytics.com
- OpenCV Documentation: https://docs.opencv.org

EOF

# Create archive
echo ""
echo "Creating compressed archive..."
tar -czf "${EXPORT_DIR}.tar.gz" "$EXPORT_DIR"

# Calculate size
SIZE=$(du -h "${EXPORT_DIR}.tar.gz" | cut -f1)

echo ""
echo "========================================="
echo "Export Complete!"
echo "========================================="
echo ""
echo "Package created:"
echo "  Directory: $EXPORT_DIR/"
echo "  Archive:   ${EXPORT_DIR}.tar.gz ($SIZE)"
echo ""
echo "Transfer to target system:"
echo "  scp ${EXPORT_DIR}.tar.gz user@host:~/"
echo ""
echo "Or copy to USB drive:"
echo "  cp ${EXPORT_DIR}.tar.gz /media/usb/"
echo ""
echo "On target system:"
echo "  tar -xzf ${EXPORT_DIR}.tar.gz"
echo "  cd ${EXPORT_DIR}/"
echo "  ./install.sh"
echo "  python3 yolo_detector_standalone.py --display"
echo ""
