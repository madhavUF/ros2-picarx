#!/bin/bash

# Fixed ROS2 PiCar-X Startup Script for Raspberry Pi OS Bookworm
# Addresses Python 3.11 compatibility and ROS key issues

set -e

# Logging function
log() {
    echo "$(date '+%Y-%m-%d %H:%M:%S') - $1"
}

log "Starting ROS2 PiCar-X setup for Raspberry Pi OS Bookworm..."

# Function to retry commands
retry_command() {
    local max_attempts=3
    local delay=2
    local command="$1"
    local description="$2"
    
    for ((i=1; i<=max_attempts; i++)); do
        log "Attempt $i/$max_attempts: $description"
        if eval "$command"; then
            log "Success: $description"
            return 0
        else
            log "Failed attempt $i/$max_attempts: $description"
            if [ $i -lt $max_attempts ]; then
                log "Retrying in $delay seconds..."
                sleep $delay
                delay=$((delay * 2))
            fi
        fi
    done
    
    log "ERROR: Failed after $max_attempts attempts: $description"
    return 1
}

# Check if running as root/sudo
if [ "$EUID" -ne 0 ]; then
    log "ERROR: This script needs to be run with sudo privileges"
    exit 1
fi

# 1. Fix ROS GPG key issue first
log "Fixing ROS repository GPG key..."
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2.list

# 2. Update package lists
log "Updating apt packages..."
retry_command "apt update -y" "Updating package lists"

# 3. Install essential packages (excluding problematic ROS packages for now)
log "Installing essential packages..."
ESSENTIAL_PACKAGES="python3-pip python3-dev nano git curl build-essential python3-colcon-common-extensions python3-gpiozero portaudio19-dev python3-opencv v4l-utils"

retry_command "apt install -y $ESSENTIAL_PACKAGES" "Installing essential packages"

# 4. Check Python version and warn about compatibility
PYTHON_VERSION=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
log "Detected Python version: $PYTHON_VERSION"

if [ "$PYTHON_VERSION" != "3.10" ]; then
    log "WARNING: ROS Iron precompiled packages expect Python 3.10, but you have Python $PYTHON_VERSION"
    log "We'll skip ROS package installation and build from source instead"
    SKIP_ROS_PACKAGES=true
else
    log "Python 3.10 detected - compatible with ROS Iron packages"
    SKIP_ROS_PACKAGES=false
fi

# 5. Install ROS packages only if Python version is compatible
if [ "$SKIP_ROS_PACKAGES" = false ]; then
    log "Installing ROS Iron packages..."
    retry_command "apt install -y ros-iron-cv-bridge ros-iron-image-transport" "Installing ROS packages"
else
    log "Skipping ROS package installation due to Python version incompatibility"
fi

# 6. Verify camera device access
log "Verifying camera device access..."
if ls /dev/video* >/dev/null 2>&1; then
    log "Video devices found: $(ls /dev/video*)"
else
    log "WARNING: No video devices found - camera may not be connected"
fi

# 7. Ensure pip3 is available
log "Ensuring pip3 is available..."
if ! command -v pip3 &> /dev/null; then
    log "pip3 not found, attempting to install manually"
    retry_command "curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py && python3 get-pip.py" "Installing pip3 manually"
    rm -f get-pip.py
fi

# 8. Install Python dependencies (with system package override)
log "Installing Python dependencies..."
PIP_PACKAGES="readchar smbus2 pyaudio gpiozero opencv-python"
for package in $PIP_PACKAGES; do
    retry_command "pip3 install --upgrade --break-system-packages $package" "Installing/upgrading $package"
done

# 9. Install robot_hat library
log "Installing robot_hat library..."
if [ ! -d "/tmp/robot-hat" ]; then
    retry_command "git clone https://github.com/sunfounder/robot-hat.git /tmp/robot-hat" "Cloning robot-hat repository"
fi
cd /tmp/robot-hat
retry_command "pip3 install . --force-reinstall --break-system-packages" "Installing robot-hat library"

# 10. Install picar-x library
log "Installing picar-x library..."
PICAR_DIR="/ros2_ws/picar-x"
if [ ! -d "$PICAR_DIR" ]; then
    log "Cloning picar-x repository..."
    cd /ros2_ws
    retry_command "git clone https://github.com/sunfounder/picar-x.git" "Cloning picar-x repository"
fi

cd "$PICAR_DIR"
retry_command "pip3 install . --force-reinstall --break-system-packages" "Installing picar-x library"

# 11. Handle picarx.py patching
log "Patching os.getlogin() issue in picarx.py..."
PICARX_PATH=$(python3 -c "import picarx, os; print(os.path.dirname(picarx.__file__))" 2>/dev/null || echo "")

if [ -n "$PICARX_PATH" ] && [ -f "$PICARX_PATH/picarx.py" ]; then
    log "Found picarx.py at: $PICARX_PATH/picarx.py"
    
    # Create backup
    cp "$PICARX_PATH/picarx.py" "$PICARX_PATH/picarx.py.backup"
    
    # Apply patch if not already applied
    if grep -q "os.getlogin()" "$PICARX_PATH/picarx.py"; then
        sed -i 's/os.getlogin()/os.getenv("USER", "pi")/g' "$PICARX_PATH/picarx.py"
        log "Applied getlogin() patch"
    else
        log "getlogin() patch already applied or not needed"
    fi
else
    log "WARNING: picarx.py not found, skipping patch"
fi

# 12. Configure system for Raspberry Pi
log "Configuring system for Raspberry Pi..."

# Enable I2C and camera interfaces
if ! grep -q "dtparam=i2c_arm=on" /boot/config.txt; then
    echo "dtparam=i2c_arm=on" >> /boot/config.txt
    log "Enabled I2C interface"
fi

if ! grep -q "dtparam=camera=on" /boot/config.txt; then
    echo "dtparam=camera=on" >> /boot/config.txt
    log "Enabled camera interface"
fi

# Add user to required groups
for group in i2c spi gpio video; do
    if groups pi 2>/dev/null | grep -q $group; then
        log "User 'pi' already in $group group"
    else
        usermod -a -G $group pi 2>/dev/null && log "Added user 'pi' to $group group" || log "WARNING: Could not add user to $group group"
    fi
done

# 13. Handle ROS2 environment setup
if [ "$SKIP_ROS_PACKAGES" = false ] && [ -f "/opt/ros/iron/setup.bash" ]; then
    log "Setting up ROS2 Iron environment..."
    source /opt/ros/iron/setup.bash
    
    # 14. Build ROS2 workspace if it exists
    if [ -d "/ros2_ws/src" ]; then
        log "Building ROS2 workspace..."
        cd /ros2_ws
        
        # Clean previous build if it failed
        if [ -d "build" ] && [ ! -f "install/setup.bash" ]; then
            log "Cleaning previous failed build..."
            rm -rf build install log
        fi
        
        # Build with timeout
        log "Building workspace (this may take a while)..."
        timeout 600 colcon build --parallel-workers 2 || {
            log "Build failed or timed out, trying with single worker..."
            timeout 900 colcon build --parallel-workers 1
        }
    else
        log "No ROS2 workspace source directory found at /ros2_ws/src"
    fi
else
    log "Skipping ROS2 workspace build (ROS not properly installed or no workspace)"
fi

# 15. Create environment setup script
log "Creating environment setup script..."
cat > /home/pi/setup_picar_env.sh << 'EOF'
#!/bin/bash
# PiCar-X Environment Setup Script

# Source ROS2 if available
if [ -f "/opt/ros/iron/setup.bash" ]; then
    source /opt/ros/iron/setup.bash
    echo "Sourced ROS2 Iron environment"
fi

# Source workspace if available
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source /ros2_ws/install/setup.bash
    echo "Sourced ROS2 workspace"
fi

# Add local Python packages to path
PYTHON_VERSION=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
LOCAL_PACKAGES="/usr/local/lib/python${PYTHON_VERSION}/dist-packages"
if [ -d "$LOCAL_PACKAGES" ]; then
    export PYTHONPATH="$LOCAL_PACKAGES:$PYTHONPATH"
fi

echo "PiCar-X environment ready!"
EOF

chmod +x /home/pi/setup_picar_env.sh
chown pi:pi /home/pi/setup_picar_env.sh

# 16. Final verification
log "Performing final verification..."

# Test Python imports
python3 -c "import picarx; print('picarx import: OK')" || log "WARNING: picarx import failed"
python3 -c "import robot_hat; print('robot_hat import: OK')" || log "WARNING: robot_hat import failed"

# Check if ROS2 is available
if command -v ros2 >/dev/null 2>&1; then
    log "ROS2 command: OK"
    if [ -n "$ROS_DISTRO" ]; then
        log "ROS_DISTRO: $ROS_DISTRO"
    fi
else
    log "WARNING: ROS2 command not available in PATH"
fi

# Cleanup temporary files
rm -rf /tmp/robot-hat

log "✅ Setup completed!"
log ""
log "IMPORTANT NOTES:"
log "1. A reboot is recommended to ensure all group memberships take effect"
log "2. To set up the environment in any new terminal session, run:"
log "   source /home/pi/setup_picar_env.sh"
log "3. If using ROS2, you may need to build packages from source due to Python version compatibility"
log ""

if [ "$SKIP_ROS_PACKAGES" = true ]; then
    log "⚠️  ROS Iron precompiled packages were skipped due to Python version incompatibility"
    log "   Consider using ROS2 Humble (compatible with Python 3.11) or building ROS from source"
fi

exit 0
