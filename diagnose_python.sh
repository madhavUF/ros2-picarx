#!/bin/bash
echo "=== Python Environment Diagnosis ==="

echo "1. Python version:"
python3 --version

echo "2. Python executable location:"
which python3

echo "3. Python path:"
python3 -c "import sys; print('\n'.join(sys.path))"

echo "4. Pip location:"
which pip3

echo "5. Site-packages locations:"
python3 -c "import site; print('\n'.join(site.getsitepackages()))"

echo "6. Check if robot_hat is in site-packages:"
find /usr -name "*robot_hat*" 2>/dev/null || echo "robot_hat not found in /usr"

echo "7. Check pip list:"
pip3 list | grep robot || echo "robot_hat not in pip list"

echo "8. Try importing with verbose mode:"
python3 -v -c "import robot_hat" 2>&1 | tail -10 || echo "Import failed"

echo "=== End Diagnosis ==="
