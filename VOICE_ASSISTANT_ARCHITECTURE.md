# PiCar-X Voice Assistant Architecture

## Overview

Transform the PiCar-X robot into a conversational AI assistant that can:
- Listen to voice commands and questions
- Respond with natural speech
- Execute physical actions (move, look, scan)
- Describe what it sees
- Navigate to locations on command

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           VOICE ASSISTANT SYSTEM                            │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────────────────────┐  │
│  │  Microphone  │───>│  STT Node    │───>│                              │  │
│  │  (USB/I2S)   │    │  (Whisper)   │    │                              │  │
│  └──────────────┘    └──────────────┘    │   CONVERSATION ORCHESTRATOR  │  │
│                                          │                              │  │
│  ┌──────────────┐    ┌──────────────┐    │   - Claude API Client        │  │
│  │   Speaker    │<───│  TTS Node    │<───│   - Tool Call Dispatcher     │  │
│  │  (USB/I2S)   │    │  (Piper/11L) │    │   - Context Management       │  │
│  └──────────────┘    └──────────────┘    │   - State Machine            │  │
│                                          │                              │  │
│                                          └──────────────┬───────────────┘  │
│                                                         │                  │
│                                                         │ Tool Calls       │
│                                                         ▼                  │
│  ┌──────────────────────────────────────────────────────────────────────┐  │
│  │                        ROS2 MCP GATEWAY                              │  │
│  │                                                                      │  │
│  │  Tools Exposed:                                                      │  │
│  │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐    │  │
│  │  │ move_robot  │ │ look_at     │ │ scan_room   │ │describe_scene│   │  │
│  │  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘    │  │
│  │  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐ ┌─────────────┐    │  │
│  │  │ get_distance│ │ stop        │ │ follow_person││ go_to_object│    │  │
│  │  └─────────────┘ └─────────────┘ └─────────────┘ └─────────────┘    │  │
│  └──────────────────────────────────────────────────────────────────────┘  │
│                                          │                                 │
│                                          │ ROS2 Topics/Services            │
│                                          ▼                                 │
├─────────────────────────────────────────────────────────────────────────────┤
│                        EXISTING ROS2 ROBOT STACK                           │
│                                                                             │
│  ┌────────────────┐  ┌────────────────┐  ┌────────────────┐                │
│  │ YOLO Detector  │  │ Robot Controller│ │ Camera Servo   │                │
│  │ /vision/detect │  │ /control/robot │  │ /control/camera│                │
│  └────────────────┘  └────────────────┘  └────────────────┘                │
│                                                                             │
│  ┌────────────────┐  ┌────────────────┐  ┌────────────────┐                │
│  │ Person Tracker │  │ Room Scanner   │  │ Sensor Publisher│               │
│  │ /tracking/*    │  │ Mission Control│  │ /sensor/*      │                │
│  └────────────────┘  └────────────────┘  └────────────────┘                │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

## Component Details

### 1. Voice Input Node (`voice_input_node.py`)

Captures audio from microphone and performs voice activity detection (VAD).

**Topics:**
- Publishes: `/voice/audio_chunk` (AudioChunk) - raw audio data
- Publishes: `/voice/speech_detected` (Bool) - VAD trigger

**Features:**
- USB microphone or I2S MEMS mic support
- WebRTC VAD for speech detection
- Configurable sample rate and chunk size
- Wake word detection (optional): "Hey PiCar"

### 2. Speech-to-Text Node (`stt_node.py`)

Converts speech audio to text using Whisper (local) or cloud API.

**Topics:**
- Subscribes: `/voice/audio_chunk`
- Publishes: `/voice/transcription` (String) - transcribed text
- Publishes: `/voice/is_listening` (Bool) - listening state

**Backends:**
- Local: `faster-whisper` (optimized for Pi)
- Cloud: OpenAI Whisper API
- Streaming: WebSocket to cloud STT

### 3. Text-to-Speech Node (`tts_node.py`)

Converts text responses to speech audio.

**Topics:**
- Subscribes: `/voice/response_text` (String)
- Publishes: `/voice/audio_output` (AudioChunk)
- Publishes: `/voice/is_speaking` (Bool)

**Backends:**
- Local: `piper-tts` (fast, runs on Pi)
- Cloud: ElevenLabs API (natural voices)
- Cloud: OpenAI TTS API

### 4. Conversation Orchestrator (`conversation_node.py`)

The brain - manages conversation flow and AI integration.

**Topics:**
- Subscribes: `/voice/transcription`
- Subscribes: `/voice/is_speaking`
- Publishes: `/voice/response_text`
- Publishes: `/assistant/state` (String: idle/listening/thinking/speaking/acting)

**Features:**
- Claude API integration with tool calling
- Conversation context management (rolling window)
- Personality/system prompt configuration
- Interruptible responses
- Action queuing and execution

### 5. ROS2 MCP Gateway (`ros2_mcp_server.py`)

Exposes robot capabilities as MCP tools for the LLM.

**MCP Tools:**

```python
# Movement Tools
move_robot(direction: str, distance: float, speed: float)
  # direction: "forward", "backward", "left", "right"
  # distance: meters (estimated from time)
  # speed: 0-100

turn_robot(angle: float, direction: str)
  # angle: degrees
  # direction: "left", "right"

stop_robot()
  # Emergency stop

# Camera Tools
look_at(pan: float, tilt: float)
  # pan: -90 to 90 degrees
  # tilt: -30 to 90 degrees

look_direction(direction: str)
  # direction: "left", "right", "up", "down", "center", "forward"

# Vision Tools
describe_scene() -> str
  # Returns natural language description of what camera sees

get_detections() -> list[Detection]
  # Returns list of detected objects with positions

find_object(object_name: str) -> ObjectLocation | None
  # Searches for specific object, returns location if found

# Sensor Tools
get_distance() -> float
  # Returns ultrasonic distance in cm

get_battery_level() -> float
  # Returns battery percentage (if available)

# Mission Tools
scan_room() -> RoomReport
  # Executes room scan mission, returns detected objects

follow_person(duration: float)
  # Enables person following for specified duration

go_to_object(object_name: str)
  # Navigates toward detected object
```

### 6. Vision Language Node (`vlm_node.py`)

Provides scene understanding using a Vision-Language Model.

**Topics:**
- Subscribes: `/vision/image_raw` (Image)
- Services: `/vlm/describe_scene` (DescribeScene.srv)
- Services: `/vlm/answer_question` (VLMQuery.srv)

**Backends:**
- Local: `moondream2` or `SmolVLM` (small, fast)
- Cloud: Claude Vision API
- Cloud: GPT-4V API

## Message Definitions

### New Messages

```
# AudioChunk.msg
std_msgs/Header header
int32 sample_rate
int32 channels
uint8[] data
float32 duration_seconds

# AssistantState.msg
std_msgs/Header header
string state  # idle, listening, thinking, speaking, acting
string current_action  # description of current action
float32 action_progress  # 0.0 to 1.0

# VoiceCommand.msg
std_msgs/Header header
string transcription
float32 confidence
string intent  # move, look, describe, question, other
string[] entities  # extracted entities
```

### New Services

```
# DescribeScene.srv
---
string description
string[] objects_detected
float32 confidence

# VLMQuery.srv
string question
---
string answer
float32 confidence

# ExecuteCommand.srv
string command_type
string[] parameters
---
bool success
string result
string error_message
```

## Data Flow

### Voice Command Flow

```
User speaks: "Go check the kitchen"
        │
        ▼
┌───────────────────┐
│ Microphone/VAD    │  Audio capture + voice activity detection
└─────────┬─────────┘
          │ /voice/audio_chunk
          ▼
┌───────────────────┐
│ STT Node          │  Whisper transcription
└─────────┬─────────┘
          │ /voice/transcription: "Go check the kitchen"
          ▼
┌───────────────────┐
│ Orchestrator      │  Send to Claude with tools
│                   │
│ Claude Response:  │
│ [tool_call:       │
│   move_robot(     │
│     "forward",    │
│     2.0, 30)]     │
│ "I'll go check    │
│  the kitchen"     │
└─────────┬─────────┘
          │
    ┌─────┴─────┐
    │           │
    ▼           ▼
┌────────┐  ┌────────────┐
│MCP     │  │TTS Node    │
│Gateway │  │            │
└────┬───┘  └─────┬──────┘
     │            │
     ▼            ▼
┌────────┐  ┌────────────┐
│Robot   │  │Speaker     │
│Moves   │  │"I'll go    │
│Forward │  │ check..."  │
└────────┘  └────────────┘
```

## Configuration

### `assistant_config.yaml`

```yaml
assistant:
  name: "PiCar"
  wake_word: "hey picar"
  personality: |
    You are PiCar, a friendly wheeled robot assistant. You can move around,
    look at things with your camera, and describe what you see. You're
    helpful, curious, and enjoy exploring. Keep responses concise since
    you'll be speaking them aloud.

voice:
  stt:
    backend: "whisper_local"  # whisper_local, whisper_api, google
    model: "base.en"
    language: "en"
  tts:
    backend: "piper"  # piper, elevenlabs, openai
    voice: "en_US-lessac-medium"
    speed: 1.0
  vad:
    aggressiveness: 2  # 0-3
    min_speech_duration: 0.5
    silence_duration: 0.8

llm:
  provider: "anthropic"  # anthropic, openai
  model: "claude-sonnet-4-20250514"
  max_tokens: 500
  temperature: 0.7

hardware:
  microphone:
    device: "plughw:1,0"  # or "default"
    sample_rate: 16000
    channels: 1
  speaker:
    device: "plughw:0,0"
    sample_rate: 22050
```

## Launch Files

### `voice_assistant.launch.py`

Launches the complete voice assistant system:

1. Core robot nodes (from existing package)
2. Voice I/O nodes
3. Conversation orchestrator
4. ROS2 MCP gateway
5. VLM node (optional)

### `assistant_sim.launch.py`

Simulation mode for testing without hardware:
- Mock audio input from file or text
- Console output instead of speaker
- Simulated robot movements

## Hardware Requirements

### Minimum (Raspberry Pi 4):
- USB microphone (or ReSpeaker 2-mic HAT)
- USB speaker or 3.5mm audio out
- Existing PiCar-X hardware

### Recommended:
- ReSpeaker 4-mic Array (better voice pickup)
- Dedicated speaker with amplifier
- USB sound card (better audio quality)

## Dependencies

### Python Packages:
```
# Voice
faster-whisper  # Local STT
piper-tts       # Local TTS
webrtcvad       # Voice activity detection
pyaudio         # Audio I/O
sounddevice     # Alternative audio I/O

# LLM
anthropic       # Claude API
openai          # OpenAI API (optional)

# Vision-Language
transformers    # For local VLM
torch           # PyTorch for inference

# MCP
mcp             # MCP SDK
```

### System Packages:
```bash
# Audio
sudo apt install portaudio19-dev python3-pyaudio
sudo apt install libsndfile1

# For Piper TTS
sudo apt install libonnxruntime
```

## Implementation Phases

### Phase 1: Basic Voice Loop
- [ ] Audio capture node
- [ ] STT node (cloud API first)
- [ ] TTS node (cloud API first)
- [ ] Basic orchestrator (echo test)

### Phase 2: LLM Integration
- [ ] Claude API integration
- [ ] Tool definitions
- [ ] ROS2 MCP gateway (basic tools)
- [ ] End-to-end voice command

### Phase 3: Robot Actions
- [ ] Movement tools
- [ ] Camera tools
- [ ] Sensor tools
- [ ] Mission tools

### Phase 4: Vision-Language
- [ ] VLM node
- [ ] Scene description
- [ ] Object finding
- [ ] Visual question answering

### Phase 5: Polish
- [ ] Local STT (Whisper)
- [ ] Local TTS (Piper)
- [ ] Wake word detection
- [ ] Conversation improvements
- [ ] Error handling and recovery
