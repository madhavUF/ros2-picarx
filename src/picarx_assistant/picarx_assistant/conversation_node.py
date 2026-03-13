#!/usr/bin/env python3
"""
Conversation Orchestrator Node - The brain of the voice assistant.

This node:
1. Receives transcribed speech from STT
2. Sends messages to Claude API with robot tools
3. Executes tool calls through the MCP gateway
4. Sends responses to TTS for speech output
"""

import os
import json
import asyncio
from typing import Optional
from dataclasses import dataclass, field
from collections import deque
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Bool

# Try to import Anthropic
try:
    import anthropic
    ANTHROPIC_AVAILABLE = True
except ImportError:
    ANTHROPIC_AVAILABLE = False
    print("Warning: anthropic package not installed. Install with: pip install anthropic")


class AssistantState(Enum):
    IDLE = "idle"
    LISTENING = "listening"
    THINKING = "thinking"
    SPEAKING = "speaking"
    ACTING = "acting"


@dataclass
class ConversationMessage:
    """A message in the conversation history."""
    role: str  # "user" or "assistant"
    content: str
    tool_calls: list = field(default_factory=list)
    tool_results: list = field(default_factory=list)


class ConversationOrchestrator(Node):
    """
    Orchestrates conversation between user, Claude API, and robot.

    This node manages:
    - Conversation history and context
    - Claude API communication
    - Tool call execution
    - Response generation
    """

    def __init__(self):
        super().__init__('conversation_orchestrator')

        # Declare parameters
        self.declare_parameter('api_key', '')
        self.declare_parameter('model', 'claude-sonnet-4-20250514')
        self.declare_parameter('max_tokens', 500)
        self.declare_parameter('temperature', 0.7)
        self.declare_parameter('max_history_length', 20)
        self.declare_parameter('system_prompt', '')

        # Get parameters
        self.api_key = self.get_parameter('api_key').value or os.environ.get('ANTHROPIC_API_KEY', '')
        self.model = self.get_parameter('model').value
        self.max_tokens = self.get_parameter('max_tokens').value
        self.temperature = self.get_parameter('temperature').value
        self.max_history = self.get_parameter('max_history_length').value

        # System prompt for the robot personality
        default_system = """You are PiCar, a friendly and helpful wheeled robot assistant. You can move around, look at things with your camera, and describe what you see.

Your capabilities:
- Move forward, backward, turn left/right
- Pan and tilt your camera to look around
- Detect and identify objects using computer vision (YOLO)
- Measure distance to obstacles with ultrasonic sensor
- Scan your surroundings

Personality:
- Friendly and enthusiastic about helping
- Curious about your environment
- Keep responses concise (you'll be speaking them aloud)
- Acknowledge commands before executing them
- Report what you see and find

When asked to do something:
1. Acknowledge the request
2. Use the appropriate tool(s)
3. Report the result

Examples:
- "Go check what's over there" → Use move_robot, then describe_scene
- "What do you see?" → Use describe_scene or get_detections
- "Find the cup" → Use find_object
- "Look left" → Use look_direction
- "How far is the wall?" → Use get_distance"""

        custom_system = self.get_parameter('system_prompt').value
        self.system_prompt = custom_system if custom_system else default_system

        # Initialize Anthropic client
        self.client = None
        if ANTHROPIC_AVAILABLE and self.api_key:
            self.client = anthropic.Anthropic(api_key=self.api_key)
            self.get_logger().info('Anthropic client initialized')
        else:
            self.get_logger().warn('No API key or anthropic not installed - running in mock mode')

        # State
        self.state = AssistantState.IDLE
        self.conversation_history: deque[ConversationMessage] = deque(maxlen=self.max_history)
        self.is_processing = False

        # MCP Gateway reference (for tool execution)
        self.mcp_gateway = None
        self.tool_definitions = []

        # QoS
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Publishers
        self.response_pub = self.create_publisher(
            String,
            '/voice/response_text',
            qos
        )
        self.state_pub = self.create_publisher(
            String,
            '/assistant/state',
            qos
        )
        self.action_pub = self.create_publisher(
            String,
            '/assistant/current_action',
            qos
        )

        # Subscribers
        self.transcription_sub = self.create_subscription(
            String,
            '/voice/transcription',
            self._transcription_callback,
            qos
        )
        self.speaking_sub = self.create_subscription(
            Bool,
            '/voice/is_speaking',
            self._speaking_callback,
            qos
        )

        # Tool execution subscriber (from MCP gateway)
        self.tool_result_sub = self.create_subscription(
            String,
            '/assistant/tool_result',
            self._tool_result_callback,
            qos
        )

        # Tool execution publisher
        self.tool_call_pub = self.create_publisher(
            String,
            '/assistant/tool_call',
            qos
        )

        # Timer for processing (allows async-like behavior)
        self.pending_input: Optional[str] = None
        self.process_timer = self.create_timer(0.1, self._process_loop)

        self._set_state(AssistantState.IDLE)
        self.get_logger().info('Conversation orchestrator initialized')

    def set_mcp_gateway(self, gateway):
        """Set the MCP gateway for direct tool execution."""
        self.mcp_gateway = gateway
        self.tool_definitions = gateway.get_tool_definitions()
        self.get_logger().info(f'MCP gateway connected with {len(self.tool_definitions)} tools')

    def _set_state(self, state: AssistantState, action: str = ""):
        """Update and publish the assistant state."""
        self.state = state
        state_msg = String()
        state_msg.data = state.value
        self.state_pub.publish(state_msg)

        if action:
            action_msg = String()
            action_msg.data = action
            self.action_pub.publish(action_msg)

    def _transcription_callback(self, msg: String):
        """Handle incoming transcription from STT."""
        text = msg.data.strip()
        if not text:
            return

        self.get_logger().info(f'Received transcription: "{text}"')

        # Queue for processing
        if not self.is_processing:
            self.pending_input = text
        else:
            self.get_logger().warn('Already processing, ignoring input')

    def _speaking_callback(self, msg: Bool):
        """Handle TTS speaking state changes."""
        if msg.data:
            self._set_state(AssistantState.SPEAKING)
        elif self.state == AssistantState.SPEAKING:
            self._set_state(AssistantState.IDLE)

    def _tool_result_callback(self, msg: String):
        """Handle tool execution results from MCP gateway."""
        # This is used when tool execution is async
        pass

    def _process_loop(self):
        """Main processing loop - handles pending input."""
        if self.pending_input and not self.is_processing:
            text = self.pending_input
            self.pending_input = None
            self._process_input(text)

    def _process_input(self, user_input: str):
        """Process user input through Claude API."""
        self.is_processing = True
        self._set_state(AssistantState.THINKING, "Processing your request...")

        # Add to history
        self.conversation_history.append(
            ConversationMessage(role="user", content=user_input)
        )

        try:
            if self.client:
                response = self._call_claude(user_input)
            else:
                response = self._mock_response(user_input)

            # Publish response for TTS
            response_msg = String()
            response_msg.data = response
            self.response_pub.publish(response_msg)

            # Add to history
            self.conversation_history.append(
                ConversationMessage(role="assistant", content=response)
            )

            self._set_state(AssistantState.SPEAKING, response)

        except Exception as e:
            self.get_logger().error(f'Error processing input: {e}')
            error_response = "I'm sorry, I encountered an error. Please try again."
            response_msg = String()
            response_msg.data = error_response
            self.response_pub.publish(response_msg)

        finally:
            self.is_processing = False

    def _call_claude(self, user_input: str) -> str:
        """Call Claude API with tools and handle response."""

        # Build messages from history
        messages = []
        for msg in self.conversation_history:
            messages.append({
                "role": msg.role,
                "content": msg.content
            })

        # Add current input
        messages.append({
            "role": "user",
            "content": user_input
        })

        # Convert tool definitions to Claude format
        tools = self._convert_tools_to_claude_format()

        try:
            # Call Claude API
            response = self.client.messages.create(
                model=self.model,
                max_tokens=self.max_tokens,
                system=self.system_prompt,
                tools=tools if tools else None,
                messages=messages
            )

            # Process response
            final_response = ""
            tool_results = []

            for content_block in response.content:
                if content_block.type == "text":
                    final_response += content_block.text
                elif content_block.type == "tool_use":
                    # Execute tool
                    tool_name = content_block.name
                    tool_input = content_block.input

                    self.get_logger().info(f'Executing tool: {tool_name}({tool_input})')
                    self._set_state(AssistantState.ACTING, f"Executing {tool_name}...")

                    result = self._execute_tool(tool_name, tool_input)
                    tool_results.append({
                        "tool_use_id": content_block.id,
                        "result": result
                    })

            # If there were tool calls, send results back to Claude
            if tool_results and response.stop_reason == "tool_use":
                # Continue conversation with tool results
                messages.append({"role": "assistant", "content": response.content})

                tool_result_content = []
                for tr in tool_results:
                    tool_result_content.append({
                        "type": "tool_result",
                        "tool_use_id": tr["tool_use_id"],
                        "content": json.dumps(tr["result"])
                    })

                messages.append({"role": "user", "content": tool_result_content})

                # Get final response
                final_response_obj = self.client.messages.create(
                    model=self.model,
                    max_tokens=self.max_tokens,
                    system=self.system_prompt,
                    tools=tools,
                    messages=messages
                )

                for block in final_response_obj.content:
                    if block.type == "text":
                        final_response = block.text
                        break

            return final_response if final_response else "I completed the action."

        except Exception as e:
            self.get_logger().error(f'Claude API error: {e}')
            raise

    def _convert_tools_to_claude_format(self) -> list:
        """Convert MCP tool definitions to Claude API format."""
        if not self.tool_definitions:
            return []

        claude_tools = []
        for tool in self.tool_definitions:
            claude_tools.append({
                "name": tool["name"],
                "description": tool["description"],
                "input_schema": tool["input_schema"]
            })

        return claude_tools

    def _execute_tool(self, tool_name: str, arguments: dict) -> dict:
        """Execute a tool through the MCP gateway."""
        if self.mcp_gateway:
            return self.mcp_gateway.execute_tool(tool_name, arguments)
        else:
            # Publish tool call for external execution
            call_msg = String()
            call_msg.data = json.dumps({
                "tool": tool_name,
                "arguments": arguments
            })
            self.tool_call_pub.publish(call_msg)

            # Return mock result for now
            return {"success": True, "message": f"Tool {tool_name} would be executed"}

    def _mock_response(self, user_input: str) -> str:
        """Generate a mock response when API is not available."""
        user_lower = user_input.lower()

        if "forward" in user_lower or "go" in user_lower:
            if self.mcp_gateway:
                self.mcp_gateway.execute_tool("move_robot", {"direction": "forward", "duration": 1.0})
            return "Moving forward!"

        elif "back" in user_lower:
            if self.mcp_gateway:
                self.mcp_gateway.execute_tool("move_robot", {"direction": "backward", "duration": 1.0})
            return "Moving backward!"

        elif "left" in user_lower:
            if self.mcp_gateway:
                self.mcp_gateway.execute_tool("look_direction", {"direction": "left"})
            return "Looking left!"

        elif "right" in user_lower:
            if self.mcp_gateway:
                self.mcp_gateway.execute_tool("look_direction", {"direction": "right"})
            return "Looking right!"

        elif "stop" in user_lower:
            if self.mcp_gateway:
                self.mcp_gateway.execute_tool("stop_robot", {})
            return "Stopping!"

        elif "see" in user_lower or "what" in user_lower:
            if self.mcp_gateway:
                result = self.mcp_gateway.execute_tool("describe_scene", {})
                return result.get("description", "I'm not sure what I see.")
            return "I can see my surroundings."

        elif "distance" in user_lower or "far" in user_lower:
            if self.mcp_gateway:
                result = self.mcp_gateway.execute_tool("get_distance", {})
                return result.get("message", "I'm not sure of the distance.")
            return "I can measure distances with my sensor."

        elif "hello" in user_lower or "hi" in user_lower:
            return "Hello! I'm PiCar, your robot assistant. How can I help you today?"

        elif "help" in user_lower:
            return "I can move around, look at things, and describe what I see. Try saying 'go forward', 'look left', or 'what do you see'!"

        else:
            return f"I heard you say: {user_input}. I'm running without the AI backend right now, so my responses are limited."


def main(args=None):
    """Main entry point for the conversation orchestrator."""
    rclpy.init(args=args)

    orchestrator = ConversationOrchestrator()

    # Try to import and connect MCP gateway
    try:
        from picarx_assistant.ros2_mcp_server import ROS2MCPGateway
        # Note: In a real setup, these would be separate nodes
        # For integrated operation, we can create the gateway here
    except ImportError:
        pass

    try:
        rclpy.spin(orchestrator)
    except KeyboardInterrupt:
        pass
    finally:
        orchestrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
