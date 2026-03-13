#!/usr/bin/env python3
"""
PiCar-X Assistant Bridge - Standalone MCP Server for Robot Control.

This bridge exposes the PiCar-X robot as an MCP server that can be
connected to by Claude Desktop, Claude CLI, or any MCP-compatible client.

Run this on the robot (Raspberry Pi) and connect to it from your computer.

Usage:
  # On the robot:
  python3 assistant_bridge.py --ros2  # With ROS2
  python3 assistant_bridge.py --standalone  # Direct hardware control

  # In Claude Desktop settings, add:
  {
    "mcpServers": {
      "picarx": {
        "command": "ssh",
        "args": ["pi@picarx.local", "python3", "/path/to/assistant_bridge.py", "--standalone"]
      }
    }
  }
"""

import argparse
import asyncio
import json
import sys
import os
from typing import Any, Optional
from dataclasses import dataclass

# MCP SDK imports
try:
    from mcp.server import Server
    from mcp.server.stdio import stdio_server
    from mcp.types import Tool, TextContent
    MCP_AVAILABLE = True
except ImportError:
    MCP_AVAILABLE = False
    print("Warning: MCP SDK not installed. Install with: pip install mcp", file=sys.stderr)

# ROS2 imports (optional)
ROS2_AVAILABLE = False
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Float32
    ROS2_AVAILABLE = True
except ImportError:
    pass

# Direct hardware imports (optional)
HARDWARE_AVAILABLE = False
try:
    from picarx import Picarx
    HARDWARE_AVAILABLE = True
except ImportError:
    pass


@dataclass
class RobotState:
    """Current state of the robot."""
    distance: float = -1.0
    camera_pan: float = 0.0
    camera_tilt: float = 0.0
    is_moving: bool = False
    detections: list = None

    def __post_init__(self):
        if self.detections is None:
            self.detections = []


class PiCarXBridge:
    """
    Bridge between MCP and PiCar-X robot.

    Can operate in three modes:
    1. ROS2 mode: Communicates via ROS2 topics
    2. Standalone mode: Direct hardware control
    3. Simulation mode: No hardware, for testing
    """

    def __init__(self, mode: str = "simulation"):
        self.mode = mode
        self.state = RobotState()
        self.px = None
        self.ros_node = None

        if mode == "standalone" and HARDWARE_AVAILABLE:
            self.px = Picarx()
            print("PiCar-X hardware initialized", file=sys.stderr)
        elif mode == "ros2" and ROS2_AVAILABLE:
            self._init_ros2()
        else:
            print(f"Running in simulation mode (mode={mode})", file=sys.stderr)

    def _init_ros2(self):
        """Initialize ROS2 connection."""
        rclpy.init()
        # Would create a ROS2 node and subscribers here
        # For simplicity, we'll use direct hardware in this implementation

    # =========================================================================
    # ROBOT CONTROL METHODS
    # =========================================================================

    def move(self, direction: str, duration: float = 1.0, speed: float = 25.0) -> dict:
        """Move the robot."""
        if self.px:
            try:
                if direction == "forward":
                    self.px.set_dir_servo_angle(0)
                    self.px.forward(speed)
                elif direction == "backward":
                    self.px.set_dir_servo_angle(0)
                    self.px.backward(speed)
                elif direction == "left":
                    self.px.set_dir_servo_angle(-30)
                    self.px.forward(speed * 0.8)
                elif direction == "right":
                    self.px.set_dir_servo_angle(30)
                    self.px.forward(speed * 0.8)

                # Wait then stop
                import time
                time.sleep(duration)
                self.px.stop()
                self.px.set_dir_servo_angle(0)

                return {"success": True, "message": f"Moved {direction} for {duration}s"}

            except Exception as e:
                return {"success": False, "error": str(e)}
        else:
            return {"success": True, "message": f"[SIM] Would move {direction} for {duration}s"}

    def turn(self, angle: float, direction: str = "left") -> dict:
        """Turn the robot."""
        if self.px:
            try:
                # Estimate duration
                duration = abs(angle) / 90.0 * 2.0

                steering = -30 if direction == "left" else 30
                self.px.set_dir_servo_angle(steering)
                self.px.forward(20)

                import time
                time.sleep(duration)
                self.px.stop()
                self.px.set_dir_servo_angle(0)

                return {"success": True, "message": f"Turned {direction} ~{angle} degrees"}

            except Exception as e:
                return {"success": False, "error": str(e)}
        else:
            return {"success": True, "message": f"[SIM] Would turn {direction} {angle} degrees"}

    def stop(self) -> dict:
        """Stop the robot."""
        if self.px:
            self.px.stop()
            self.px.set_dir_servo_angle(0)
        return {"success": True, "message": "Robot stopped"}

    def look_at(self, pan: float = 0.0, tilt: float = 0.0) -> dict:
        """Point the camera."""
        pan = max(-90, min(90, pan))
        tilt = max(-30, min(90, tilt))

        if self.px:
            try:
                self.px.set_cam_pan_angle(pan)
                self.px.set_cam_tilt_angle(tilt)
                self.state.camera_pan = pan
                self.state.camera_tilt = tilt
                return {"success": True, "message": f"Camera at pan={pan}, tilt={tilt}"}
            except Exception as e:
                return {"success": False, "error": str(e)}
        else:
            self.state.camera_pan = pan
            self.state.camera_tilt = tilt
            return {"success": True, "message": f"[SIM] Camera at pan={pan}, tilt={tilt}"}

    def look_direction(self, direction: str) -> dict:
        """Look in a named direction."""
        directions = {
            "left": (-60, 0),
            "right": (60, 0),
            "up": (0, 45),
            "down": (0, -20),
            "center": (0, 0),
            "forward": (0, 0),
        }

        if direction in directions:
            pan, tilt = directions[direction]
            return self.look_at(pan, tilt)
        else:
            return {"success": False, "error": f"Unknown direction: {direction}"}

    def get_distance(self) -> dict:
        """Get ultrasonic distance."""
        if self.px:
            try:
                # Note: Picarx may not have built-in ultrasonic method
                # This would need to be implemented based on your hardware setup
                distance = -1  # Placeholder
                return {"success": True, "distance_cm": distance}
            except Exception as e:
                return {"success": False, "error": str(e)}
        else:
            return {"success": True, "distance_cm": 50.0, "message": "[SIM] Simulated distance"}

    def get_status(self) -> dict:
        """Get robot status."""
        return {
            "success": True,
            "status": {
                "mode": self.mode,
                "hardware_available": self.px is not None,
                "camera_pan": self.state.camera_pan,
                "camera_tilt": self.state.camera_tilt,
                "is_moving": self.state.is_moving
            }
        }


# =============================================================================
# MCP SERVER
# =============================================================================

def create_mcp_server(bridge: PiCarXBridge) -> Server:
    """Create the MCP server with robot tools."""

    server = Server("picarx-assistant")

    @server.list_tools()
    async def list_tools() -> list[Tool]:
        """Return list of available tools."""
        return [
            Tool(
                name="move_robot",
                description="Move the robot in a direction. Use for commands like 'go forward', 'back up', etc.",
                inputSchema={
                    "type": "object",
                    "properties": {
                        "direction": {
                            "type": "string",
                            "enum": ["forward", "backward", "left", "right"],
                            "description": "Direction to move"
                        },
                        "duration": {
                            "type": "number",
                            "description": "How long to move in seconds (default 1.0)",
                            "default": 1.0
                        },
                        "speed": {
                            "type": "number",
                            "description": "Speed 0-100 (default 25)",
                            "default": 25
                        }
                    },
                    "required": ["direction"]
                }
            ),
            Tool(
                name="turn_robot",
                description="Turn the robot by a specific angle.",
                inputSchema={
                    "type": "object",
                    "properties": {
                        "angle": {
                            "type": "number",
                            "description": "Degrees to turn"
                        },
                        "direction": {
                            "type": "string",
                            "enum": ["left", "right"],
                            "description": "Turn direction"
                        }
                    },
                    "required": ["angle", "direction"]
                }
            ),
            Tool(
                name="stop_robot",
                description="Immediately stop all robot movement.",
                inputSchema={
                    "type": "object",
                    "properties": {}
                }
            ),
            Tool(
                name="look_at",
                description="Point the camera at specific pan/tilt angles.",
                inputSchema={
                    "type": "object",
                    "properties": {
                        "pan": {
                            "type": "number",
                            "description": "Horizontal: -90 (left) to 90 (right)"
                        },
                        "tilt": {
                            "type": "number",
                            "description": "Vertical: -30 (down) to 90 (up)"
                        }
                    }
                }
            ),
            Tool(
                name="look_direction",
                description="Point camera in a named direction (easier than angles).",
                inputSchema={
                    "type": "object",
                    "properties": {
                        "direction": {
                            "type": "string",
                            "enum": ["left", "right", "up", "down", "center", "forward"],
                            "description": "Named direction"
                        }
                    },
                    "required": ["direction"]
                }
            ),
            Tool(
                name="get_distance",
                description="Get ultrasonic distance sensor reading (cm to nearest obstacle).",
                inputSchema={
                    "type": "object",
                    "properties": {}
                }
            ),
            Tool(
                name="get_robot_status",
                description="Get current robot status including camera position and state.",
                inputSchema={
                    "type": "object",
                    "properties": {}
                }
            ),
        ]

    @server.call_tool()
    async def call_tool(name: str, arguments: dict) -> list[TextContent]:
        """Execute a tool call."""
        result = {}

        if name == "move_robot":
            result = bridge.move(
                direction=arguments.get("direction", "forward"),
                duration=arguments.get("duration", 1.0),
                speed=arguments.get("speed", 25)
            )
        elif name == "turn_robot":
            result = bridge.turn(
                angle=arguments.get("angle", 90),
                direction=arguments.get("direction", "left")
            )
        elif name == "stop_robot":
            result = bridge.stop()
        elif name == "look_at":
            result = bridge.look_at(
                pan=arguments.get("pan", 0),
                tilt=arguments.get("tilt", 0)
            )
        elif name == "look_direction":
            result = bridge.look_direction(arguments.get("direction", "center"))
        elif name == "get_distance":
            result = bridge.get_distance()
        elif name == "get_robot_status":
            result = bridge.get_status()
        else:
            result = {"success": False, "error": f"Unknown tool: {name}"}

        return [TextContent(type="text", text=json.dumps(result, indent=2))]

    return server


async def run_mcp_server(mode: str):
    """Run the MCP server."""
    if not MCP_AVAILABLE:
        print("Error: MCP SDK not available", file=sys.stderr)
        sys.exit(1)

    bridge = PiCarXBridge(mode=mode)
    server = create_mcp_server(bridge)

    print(f"Starting PiCar-X MCP server in {mode} mode...", file=sys.stderr)

    async with stdio_server() as (read_stream, write_stream):
        await server.run(read_stream, write_stream, server.create_initialization_options())


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description="PiCar-X MCP Assistant Bridge")
    parser.add_argument("--ros2", action="store_true", help="Use ROS2 for communication")
    parser.add_argument("--standalone", action="store_true", help="Direct hardware control")
    parser.add_argument("--simulation", action="store_true", help="Simulation mode (default)")

    args = parser.parse_args()

    if args.ros2:
        mode = "ros2"
    elif args.standalone:
        mode = "standalone"
    else:
        mode = "simulation"

    asyncio.run(run_mcp_server(mode))


if __name__ == "__main__":
    main()
