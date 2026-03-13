# PiCar-X Voice Assistant Setup Guide

This guide will help you transform your PiCar-X robot into a voice-controlled AI assistant.

## Overview

The voice assistant system adds these capabilities to your robot:
- **Voice Commands**: Speak to control the robot ("Go forward", "Look left")
- **Natural Responses**: The robot speaks back using text-to-speech
- **AI Understanding**: Claude understands context and intent
- **MCP Integration**: Control via Claude Desktop or Claude CLI

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    YOUR VOICE                           │
│                        │                                │
│                        ▼                                │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────┐ │
│  │ Microphone  │→ │ STT (Whisper)│→│ Claude + Tools  │ │
│  └─────────────┘  └─────────────┘  └────────┬────────┘ │
│                                              │          │
│  ┌─────────────┐  ┌─────────────┐  ┌────────▼────────┐ │
│  │   Speaker   │← │ TTS (Piper) │← │ MCP Gateway     │ │
│  └─────────────┘  └─────────────┘  └────────┬────────┘ │
│                                              │          │
│                    ┌─────────────────────────▼────────┐ │
│                    │        ROS2 Robot Stack         │ │
│                    │  (Motors, Camera, Sensors)      │ │
│                    └─────────────────────────────────┘ │
└─────────────────────────────────────────────────────────┘
```

## Hardware Requirements

### Minimum
- Raspberry Pi 4 (4GB+ recommended)
- USB Microphone
- USB Speaker or 3.5mm audio out
- PiCar-X robot kit (assembled)

### Recommended
- ReSpeaker 2-Mic HAT or 4-Mic Array (better voice pickup)
- Powered USB speaker
- Pi Camera Module 3

## Installation

### 1. Install System Dependencies

```bash
# On the Raspberry Pi
sudo apt update
sudo apt install -y \
    portaudio19-dev \
    python3-pyaudio \
    libsndfile1 \
    espeak \
    alsa-utils \
    ffmpeg

# For ROS2 (if not already installed)
# Follow: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
```

### 2. Install Python Dependencies

```bash
# Create a virtual environment (optional but recommended)
python3 -m venv ~/picarx_env
source ~/picarx_env/bin/activate

# Install dependencies
cd ~/ros2-picarx/src/picarx_assistant
pip install -r requirements.txt

# For local STT (Whisper)
pip install faster-whisper

# For local TTS (Piper)
pip install piper-tts
# Or download binary: https://github.com/rhasspy/piper/releases
```

### 3. Install MCP SDK

```bash
pip install mcp
```

### 4. Set Up API Keys

```bash
# Add to ~/.bashrc or create .env file
export ANTHROPIC_API_KEY="your-api-key-here"

# Optional: For OpenAI STT/TTS
export OPENAI_API_KEY="your-openai-key"
```

### 5. Build the ROS2 Package

```bash
cd ~/ros2-picarx
source /opt/ros/humble/setup.bash
colcon build --packages-select picarx_assistant
source install/setup.bash
```

## Usage

### Option 1: Full Voice Assistant (On Robot)

This runs the complete system with voice input/output:

```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash
source ~/ros2-picarx/install/setup.bash

# Launch voice assistant
ros2 launch picarx_assistant voice_assistant.launch.py

# With simulation mode (no hardware):
ros2 launch picarx_assistant voice_assistant.launch.py simulation:=true
```

Then just talk to your robot!

### Option 2: MCP Bridge (Control from Claude Desktop)

This allows you to control the robot from Claude Desktop or Claude CLI:

**On the Robot:**
```bash
# Option A: Via ROS2 (full features)
ros2 launch picarx_assistant mcp_bridge.launch.py

# Option B: Standalone (direct hardware control)
python3 ~/ros2-picarx/src/picarx_assistant/picarx_assistant/assistant_bridge.py --standalone
```

**On Your Computer (Claude Desktop):**

Add to your Claude Desktop configuration (`~/Library/Application Support/Claude/claude_desktop_config.json` on Mac):

```json
{
  "mcpServers": {
    "picarx": {
      "command": "ssh",
      "args": [
        "pi@picarx.local",
        "python3",
        "/home/pi/ros2-picarx/src/picarx_assistant/picarx_assistant/assistant_bridge.py",
        "--standalone"
      ]
    }
  }
}
```

Then in Claude Desktop, you can say things like:
- "Move the robot forward"
- "Look to the left"
- "What's the distance to the nearest obstacle?"

### Option 3: Claude CLI with MCP

```bash
# Install Claude CLI if not already
npm install -g @anthropic-ai/claude-code

# Run with robot tools
claude --mcp-server "ssh pi@picarx.local python3 /path/to/assistant_bridge.py --standalone"
```

## Voice Commands

The assistant understands natural language. Try:

### Movement
- "Go forward"
- "Move backward for 2 seconds"
- "Turn left"
- "Turn around"
- "Stop"

### Camera
- "Look left"
- "Look up"
- "Look at the center"
- "Point the camera down"

### Vision
- "What do you see?"
- "Is there a person in front of you?"
- "Find the cup"
- "Describe what's around you"

### Sensors
- "How far is the wall?"
- "What's your distance reading?"

### Compound Commands
- "Go check what's in the corner"
- "Look around and tell me what you find"
- "Find the chair and go towards it"

## Configuration

Edit `config/assistant_config.yaml` to customize:

- Voice settings (STT/TTS backends, voice model)
- Robot behavior (speeds, safety distances)
- AI model settings
- Wake word

## Troubleshooting

### No Audio Input
```bash
# List audio devices
arecord -l

# Test recording
arecord -d 5 test.wav
aplay test.wav

# Set default device in ~/.asoundrc or via PulseAudio
```

### No Audio Output
```bash
# Test speakers
espeak "Hello, I am PiCar"

# Check volume
alsamixer
```

### STT Not Working
```bash
# Test Whisper locally
python3 -c "from faster_whisper import WhisperModel; print('OK')"

# Check model download
# Models are downloaded to ~/.cache/huggingface/
```

### Robot Not Moving
```bash
# Check ROS2 topics
ros2 topic list
ros2 topic echo /control/robot_command

# Test hardware directly
python3 -c "from picarx import Picarx; px = Picarx(); px.forward(20); import time; time.sleep(1); px.stop()"
```

### API Key Issues
```bash
# Verify API key is set
echo $ANTHROPIC_API_KEY

# Test API connection
python3 -c "import anthropic; c = anthropic.Anthropic(); print(c.messages.create(model='claude-sonnet-4-20250514', max_tokens=10, messages=[{'role':'user','content':'Hi'}]))"
```

## Project Structure

```
src/picarx_assistant/
├── picarx_assistant/
│   ├── __init__.py
│   ├── voice_input_node.py    # Microphone + VAD
│   ├── stt_node.py            # Speech-to-Text
│   ├── tts_node.py            # Text-to-Speech
│   ├── conversation_node.py   # Claude orchestration
│   ├── ros2_mcp_server.py     # ROS2 MCP gateway
│   ├── vlm_node.py            # Vision-Language Model
│   └── assistant_bridge.py    # Standalone MCP server
├── msg/
│   ├── AudioChunk.msg
│   ├── AssistantState.msg
│   └── VoiceCommand.msg
├── srv/
│   ├── DescribeScene.srv
│   ├── VLMQuery.srv
│   └── ExecuteCommand.srv
├── launch/
│   ├── voice_assistant.launch.py
│   └── mcp_bridge.launch.py
├── config/
│   └── assistant_config.yaml
├── package.xml
├── CMakeLists.txt
└── setup.py
```

## What's Next?

- **Wake Word**: Add "Hey PiCar" wake word detection
- **Local LLM**: Run smaller models locally on Pi
- **Gestures**: Add head nods/shakes for feedback
- **Memory**: Persistent conversation memory
- **Skills**: Custom skills ("patrol mode", "follow me")
- **Web Interface**: Browser-based control panel

## Credits

Inspired by [Reachy Mini](https://huggingface.co/spaces/pollen-robotics/Reachy_Mini) from Pollen Robotics.

Sources:
- [Reachy Mini SDK](https://github.com/pollen-robotics/reachy_mini)
- [Reachy Mini Conversation App](https://github.com/pollen-robotics/reachy_mini_conversation_app)
- [Pollen Robotics](https://www.pollen-robotics.com/reachy-mini/)
