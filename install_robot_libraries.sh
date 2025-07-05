#!/bin/bash
echo "🔧 Installing robot libraries properly..."

# Function to check if installation worked
check_install() {
    python3 -c "import $1; print('✅ $1 installed successfully')" 2>/dev/null || echo "❌ $1 installation failed"
}

echo "📍 Installing system dependencies..."
apt update -qq && apt install -y portaudio19-dev python3-dev

echo "📍 Installing pyaudio..."
pip3 install pyaudio

echo "📍 Installing robot_hat from source..."
cd /workspace/robot-hat
python3 setup.py install --force
echo "📍 Checking robot_hat installation..."
check_install "robot_hat"

# If robot_hat failed, try alternative method
if ! python3 -c "import robot_hat" 2>/dev/null; then
    echo "📍 Trying alternative robot_hat installation..."
    pip3 install -e .
    check_install "robot_hat"
fi

echo "📍 Installing picar-x..."
cd /workspace/picar-x
python3 setup.py install --force

echo "📍 Checking picar-x installation..."
check_install "picarx"

# Apply patch only if both libraries work
echo "📍 Testing imports before patching..."
if python3 -c "import robot_hat, picarx" 2>/dev/null; then
    echo "📍 Applying picarx patch..."
    python3 -c "
import picarx, os
picarx_file = picarx.__file__.replace('__init__.py', 'picarx.py')
if os.path.exists(picarx_file):
    with open(picarx_file, 'r') as f: content = f.read()
    content = content.replace('os.getlogin()', '__import__(\"getpass\").getuser()')
    with open(picarx_file, 'w') as f: f.write(content)
    print('✅ Patch applied')
else:
    print('❌ Could not find picarx.py file')
"
else
    echo "❌ Libraries not working, skipping patch"
fi

echo "📍 Final test..."
python3 -c "from picarx import Picarx; print('🚗 All libraries working!')" || echo "❌ Final test failed"

echo "📍 Updated pip list:"
pip3 list | grep -E "(robot|picar)" || echo "Libraries not showing in pip list"
