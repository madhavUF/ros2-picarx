#!/bin/bash

echo "🐳 Starting ROS 2 Docker container with mounted workspace..."

docker run -it --rm \
  --network host \
  --privileged \
  -v /dev:/dev \
  -v ~/ros2_ws:/ros2_ws \
  -w /ros2_ws \
  ros:iron \
  bash -c "
    echo '📦 Updating apt...'
    apt update -y

    echo '🔧 Installing essential packages...'
    apt install -y python3-pip python3-dev nano git curl build-essential \
                   python3-colcon-common-extensions python3-gpiozero \
                   portaudio19-dev

    echo '🐍 Installing Python dependencies...'
    pip3 install --upgrade pip
    pip3 install readchar smbus2 pyaudio gpiozero

    echo '📁 Installing robot_hat library...'
    if [ ! -d /ros2_ws/robot-hat ]; then
      cd /ros2_ws && git clone https://github.com/sunfounder/robot-hat.git
    fi
    cd /ros2_ws/robot-hat && pip3 install . --force-reinstall

    echo '📁 Installing picar-x library...'
    if [ ! -d /ros2_ws/picar-x ]; then
      cd /ros2_ws && git clone https://github.com/sunfounder/picar-x.git
    fi
    cd /ros2_ws/picar-x && pip3 install . --force-reinstall

    echo '🩹 Patching os.getlogin() issue in picarx.py...'
    sed -i 's/os.getlogin()/__import__(\"getpass\").getuser()/g' \
      /usr/local/lib/python3.10/dist-packages/picarx/picarx.py

    echo '🛠️ Building ROS 2 workspace...'
    cd /ros2_ws
    source /opt/ros/iron/setup.bash
    colcon build

    echo '🧠 Sourcing environment and entering shell...'
    source /opt/ros/iron/setup.bash
    source install/setup.bash
    export PYTHONPATH=/usr/local/lib/python3.10/dist-packages:\$PYTHONPATH

    echo '✅ ROS 2 environment is ready!'
    exec bash
  "
