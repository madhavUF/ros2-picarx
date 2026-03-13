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
