#!/bin/bash
# Deploy PiCar-X Voice Assistant to Raspberry Pi
# Usage: ./deploy_assistant.sh [pi_address]

set -e

# Configuration
PI_USER="pi"
PI_HOST="${1:-picarx.local}"  # Pass IP/hostname as argument, or use default
PI_PATH="/home/pi/ros2-picarx"

echo "=========================================="
echo "PiCar-X Voice Assistant Deployment"
echo "=========================================="
echo "Target: ${PI_USER}@${PI_HOST}:${PI_PATH}"
echo ""

# Check SSH connectivity
echo "Checking SSH connection..."
if ! ssh -o ConnectTimeout=5 "${PI_USER}@${PI_HOST}" "echo 'Connected!'" 2>/dev/null; then
    echo "ERROR: Cannot connect to ${PI_USER}@${PI_HOST}"
    echo ""
    echo "Please ensure:"
    echo "  1. Pi is powered on and connected to network"
    echo "  2. SSH is enabled on the Pi"
    echo "  3. You can reach it (try: ping ${PI_HOST})"
    echo ""
    echo "Usage: $0 <pi_ip_or_hostname>"
    echo "Example: $0 192.168.1.100"
    exit 1
fi

echo ""
echo "Step 1: Syncing picarx_assistant package..."
rsync -avz --progress \
    --exclude '__pycache__' \
    --exclude '*.pyc' \
    --exclude '.git' \
    src/picarx_assistant/ \
    "${PI_USER}@${PI_HOST}:${PI_PATH}/src/picarx_assistant/"

echo ""
echo "Step 2: Syncing documentation and setup files..."
rsync -avz --progress \
    VOICE_ASSISTANT_ARCHITECTURE.md \
    VOICE_ASSISTANT_SETUP.md \
    "${PI_USER}@${PI_HOST}:${PI_PATH}/"

echo ""
echo "Step 3: Installing Python dependencies on Pi..."
ssh "${PI_USER}@${PI_HOST}" << 'REMOTE_SCRIPT'
cd /home/pi/ros2-picarx

echo "Installing system dependencies..."
sudo apt-get update
sudo apt-get install -y portaudio19-dev python3-pyaudio libsndfile1 espeak alsa-utils

echo "Installing Python packages..."
pip3 install --user anthropic webrtcvad mcp 2>/dev/null || pip install --user anthropic webrtcvad mcp

echo "Installing faster-whisper (this may take a while on Pi)..."
pip3 install --user faster-whisper 2>/dev/null || echo "Note: faster-whisper may need manual install"

echo "Dependencies installed!"
REMOTE_SCRIPT

echo ""
echo "Step 4: Building ROS2 package on Pi..."
ssh "${PI_USER}@${PI_HOST}" << 'REMOTE_SCRIPT'
cd /home/pi/ros2-picarx

# Source ROS2
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
elif [ -f /opt/ros/iron/setup.bash ]; then
    source /opt/ros/iron/setup.bash
else
    echo "Warning: ROS2 not found in standard locations"
fi

# Build the new package
echo "Building picarx_assistant package..."
colcon build --packages-select picarx_assistant --symlink-install 2>&1 || {
    echo "Build failed - this might be due to missing ROS2 or dependencies"
    echo "You can try building manually on the Pi"
}

echo "Build complete!"
REMOTE_SCRIPT

echo ""
echo "=========================================="
echo "Deployment Complete!"
echo "=========================================="
echo ""
echo "Next steps on the Pi:"
echo ""
echo "1. Set your API key:"
echo "   export ANTHROPIC_API_KEY='your-key-here'"
echo "   # Add to ~/.bashrc for persistence"
echo ""
echo "2. Run the voice assistant:"
echo "   source /opt/ros/humble/setup.bash"
echo "   source ~/ros2-picarx/install/setup.bash"
echo "   ros2 launch picarx_assistant voice_assistant.launch.py"
echo ""
echo "3. Or run standalone MCP bridge (for Claude Desktop):"
echo "   python3 ~/ros2-picarx/src/picarx_assistant/picarx_assistant/assistant_bridge.py --standalone"
echo ""
