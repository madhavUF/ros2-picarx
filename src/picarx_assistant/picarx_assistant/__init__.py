# PiCar-X Voice Assistant Package
"""
Voice-controlled AI assistant for PiCar-X robot with ROS2 MCP gateway.

Modules:
- voice_input_node: Audio capture and VAD
- stt_node: Speech-to-text
- tts_node: Text-to-speech
- conversation_node: LLM orchestration
- ros2_mcp_server: MCP gateway for robot control
- vlm_node: Vision-language model integration
- assistant_bridge: Standalone bridge for external MCP clients
"""

__version__ = "1.0.0"
