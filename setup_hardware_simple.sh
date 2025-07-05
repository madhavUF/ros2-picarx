#!/bin/bash
echo "🔧 Simple hardware setup..."

# Install robot_hat
echo "Installing robot_hat..."
cd /workspace/robot-hat
pip3 install .

# Test robot_hat
python3 -c "import robot_hat; print('robot_hat works')" || echo "robot_hat failed"

# Install picar-x
echo "Installing picar-x..."
cd /workspace/picar-x
pip3 install .

# Apply patch manually
echo "Applying patch..."
python3 -c "
try:
    import picarx
    import os
    picarx_file = picarx.__file__.replace('__init__.py', 'picarx.py')
    if os.path.exists(picarx_file):
        with open(picarx_file, 'r') as f:
            content = f.read()
        content = content.replace('os.getlogin()', '__import__(\"getpass\").getuser()')
        with open(picarx_file, 'w') as f:
            f.write(content)
        print('Patch applied successfully')
    else:
        print('Could not find picarx.py file')
except Exception as e:
    print(f'Patch failed: {e}')
"

# Test final result
python3 -c "from picarx import Picarx; print('Hardware ready')" || echo "Still need to fix issues"
