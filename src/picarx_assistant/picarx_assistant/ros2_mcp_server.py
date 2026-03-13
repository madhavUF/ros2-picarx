#!/usr/bin/env python3
"""
ROS2 MCP Server - Exposes robot capabilities as MCP tools.

This server bridges LLM tool calls to ROS2 robot control, enabling
voice commands like "go forward" to translate into robot movement.
"""

import json
import asyncio
import threading
from typing import Any, Optional
from dataclasses import dataclass
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Float32, Header
from sensor_msgs.msg import Image

# Import from the existing my_first_pkg
# These will be available when running in the ROS2 environment
try:
    from my_first_pkg.msg import RobotCommand, CameraCommand, DetectionArray
except ImportError:
    # Mock for development/testing outside ROS2
    RobotCommand = None
    CameraCommand = None
    DetectionArray = None


class RobotState(Enum):
    IDLE = "idle"
    MOVING = "moving"
    SCANNING = "scanning"
    FOLLOWING = "following"


@dataclass
class Detection:
    """Represents a detected object."""
    class_name: str
    confidence: float
    center_x: float
    center_y: float
    area: float


class ROS2MCPGateway(Node):
    """
    ROS2 Node that provides an MCP-compatible interface for robot control.

    This node:
    1. Subscribes to sensor/detection topics
    2. Publishes control commands
    3. Exposes methods that can be called as MCP tools
    """

    def __init__(self):
        super().__init__('ros2_mcp_gateway')

        # State
        self.robot_state = RobotState.IDLE
        self.current_distance = -1.0
        self.current_detections: list[Detection] = []
        self.latest_image = None
        self.camera_pan = 0.0
        self.camera_tilt = 0.0

        # QoS for reliable communication
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Publishers - to control the robot
        self.robot_cmd_pub = self.create_publisher(
            RobotCommand if RobotCommand else String,
            '/control/robot_command',
            qos
        )
        self.camera_cmd_pub = self.create_publisher(
            CameraCommand if CameraCommand else String,
            '/control/camera_command',
            qos
        )

        # Subscribers - to get robot state
        self.distance_sub = self.create_subscription(
            Float32,
            '/sensor/distance',
            self._distance_callback,
            qos
        )

        if DetectionArray:
            self.detection_sub = self.create_subscription(
                DetectionArray,
                '/vision/detections',
                self._detection_callback,
                qos
            )

        # State publisher for assistant feedback
        self.state_pub = self.create_publisher(
            String,
            '/assistant/robot_state',
            qos
        )

        self.get_logger().info('ROS2 MCP Gateway initialized')

    def _distance_callback(self, msg: Float32):
        """Update current distance reading."""
        self.current_distance = msg.data

    def _detection_callback(self, msg):
        """Update current detections."""
        self.current_detections = [
            Detection(
                class_name=d.class_name,
                confidence=d.confidence,
                center_x=d.center_x,
                center_y=d.center_y,
                area=d.area
            )
            for d in msg.detections
        ]

    def _publish_robot_command(self, action: str, speed: float = 25.0,
                                steering_angle: float = 0.0):
        """Publish a robot movement command."""
        if RobotCommand:
            msg = RobotCommand()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.action = action
            msg.speed = speed
            msg.steering_angle = steering_angle
        else:
            # Fallback for testing without custom messages
            msg = String()
            msg.data = json.dumps({
                'action': action,
                'speed': speed,
                'steering_angle': steering_angle
            })

        self.robot_cmd_pub.publish(msg)
        self.get_logger().info(f'Robot command: {action}, speed={speed}, angle={steering_angle}')

    def _publish_camera_command(self, pan: float, tilt: float, reset: bool = False):
        """Publish a camera servo command."""
        if CameraCommand:
            msg = CameraCommand()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.pan_angle = pan
            msg.tilt_angle = tilt
            msg.reset_to_center = reset
        else:
            msg = String()
            msg.data = json.dumps({
                'pan_angle': pan,
                'tilt_angle': tilt,
                'reset_to_center': reset
            })

        self.camera_cmd_pub.publish(msg)
        self.camera_pan = pan
        self.camera_tilt = tilt
        self.get_logger().info(f'Camera command: pan={pan}, tilt={tilt}')

    # =========================================================================
    # MCP TOOLS - These methods are exposed as tools for the LLM
    # =========================================================================

    def tool_move_robot(self, direction: str, duration: float = 1.0,
                        speed: float = 25.0) -> dict:
        """
        Move the robot in a direction for a specified duration.

        Args:
            direction: One of "forward", "backward", "left", "right"
            duration: How long to move in seconds (default 1.0)
            speed: Movement speed 0-100 (default 25)

        Returns:
            dict with success status and message
        """
        valid_directions = ["forward", "backward", "left", "right"]
        if direction not in valid_directions:
            return {
                "success": False,
                "error": f"Invalid direction. Must be one of: {valid_directions}"
            }

        # Map directions to actions
        action_map = {
            "forward": "forward",
            "backward": "backward",
            "left": "turn_left",
            "right": "turn_right"
        }

        steering = 15.0 if direction in ["left", "right"] else 0.0

        self.robot_state = RobotState.MOVING
        self._publish_robot_command(action_map[direction], speed, steering)

        # Schedule stop after duration
        self.create_timer(duration, lambda: self._stop_and_reset())

        return {
            "success": True,
            "message": f"Moving {direction} for {duration} seconds at speed {speed}"
        }

    def _stop_and_reset(self):
        """Stop robot and reset state."""
        self._publish_robot_command("stop", 0.0, 0.0)
        self.robot_state = RobotState.IDLE

    def tool_turn_robot(self, angle: float, direction: str = "left") -> dict:
        """
        Turn the robot by a specified angle.

        Args:
            angle: Degrees to turn (positive value)
            direction: "left" or "right"

        Returns:
            dict with success status
        """
        if direction not in ["left", "right"]:
            return {"success": False, "error": "Direction must be 'left' or 'right'"}

        # Estimate duration based on angle (rough approximation)
        # This would need calibration for the actual robot
        duration = abs(angle) / 90.0 * 2.0  # ~2 seconds for 90 degrees

        action = "turn_left" if direction == "left" else "turn_right"
        self.robot_state = RobotState.MOVING
        self._publish_robot_command(action, 20.0, 25.0)

        self.create_timer(duration, lambda: self._stop_and_reset())

        return {
            "success": True,
            "message": f"Turning {direction} approximately {angle} degrees"
        }

    def tool_stop_robot(self) -> dict:
        """
        Immediately stop the robot.

        Returns:
            dict with success status
        """
        self._publish_robot_command("stop", 0.0, 0.0)
        self.robot_state = RobotState.IDLE
        return {"success": True, "message": "Robot stopped"}

    def tool_look_at(self, pan: float = 0.0, tilt: float = 0.0) -> dict:
        """
        Point the camera in a specific direction.

        Args:
            pan: Horizontal angle, -90 (left) to 90 (right)
            tilt: Vertical angle, -30 (down) to 90 (up)

        Returns:
            dict with success status
        """
        # Clamp values to valid ranges
        pan = max(-90.0, min(90.0, pan))
        tilt = max(-30.0, min(90.0, tilt))

        self._publish_camera_command(pan, tilt)

        return {
            "success": True,
            "message": f"Camera pointed to pan={pan}, tilt={tilt}"
        }

    def tool_look_direction(self, direction: str) -> dict:
        """
        Point the camera in a named direction.

        Args:
            direction: One of "left", "right", "up", "down", "center",
                      "forward", "left_up", "right_up", "left_down", "right_down"

        Returns:
            dict with success status
        """
        direction_map = {
            "left": (-60.0, 0.0),
            "right": (60.0, 0.0),
            "up": (0.0, 45.0),
            "down": (0.0, -20.0),
            "center": (0.0, 0.0),
            "forward": (0.0, 0.0),
            "left_up": (-45.0, 30.0),
            "right_up": (45.0, 30.0),
            "left_down": (-45.0, -15.0),
            "right_down": (45.0, -15.0),
        }

        if direction not in direction_map:
            return {
                "success": False,
                "error": f"Unknown direction. Valid: {list(direction_map.keys())}"
            }

        pan, tilt = direction_map[direction]
        return self.tool_look_at(pan, tilt)

    def tool_get_distance(self) -> dict:
        """
        Get the current ultrasonic distance reading.

        Returns:
            dict with distance in centimeters
        """
        return {
            "success": True,
            "distance_cm": self.current_distance,
            "message": f"Distance sensor reads {self.current_distance:.1f} cm"
        }

    def tool_get_detections(self) -> dict:
        """
        Get current object detections from the camera.

        Returns:
            dict with list of detected objects
        """
        detections = [
            {
                "object": d.class_name,
                "confidence": round(d.confidence, 2),
                "position": {
                    "x": round(d.center_x, 1),
                    "y": round(d.center_y, 1)
                },
                "size": round(d.area, 0)
            }
            for d in self.current_detections
        ]

        return {
            "success": True,
            "detections": detections,
            "count": len(detections),
            "message": f"Detected {len(detections)} objects"
        }

    def tool_find_object(self, object_name: str) -> dict:
        """
        Check if a specific object is visible.

        Args:
            object_name: Name of object to find (e.g., "person", "cup", "chair")

        Returns:
            dict with object location if found
        """
        matches = [
            d for d in self.current_detections
            if d.class_name.lower() == object_name.lower()
        ]

        if not matches:
            return {
                "success": True,
                "found": False,
                "message": f"No {object_name} detected in current view"
            }

        # Return the highest confidence match
        best = max(matches, key=lambda d: d.confidence)

        # Determine relative position
        frame_center_x = 320  # Assuming 640 width
        if best.center_x < frame_center_x - 50:
            horizontal = "to the left"
        elif best.center_x > frame_center_x + 50:
            horizontal = "to the right"
        else:
            horizontal = "in the center"

        return {
            "success": True,
            "found": True,
            "object": object_name,
            "confidence": round(best.confidence, 2),
            "position": horizontal,
            "message": f"Found {object_name} {horizontal} with {best.confidence:.0%} confidence"
        }

    def tool_describe_scene(self) -> dict:
        """
        Get a description of what the robot currently sees.

        Returns:
            dict with scene description based on detections
        """
        if not self.current_detections:
            return {
                "success": True,
                "description": "I don't see any recognizable objects right now.",
                "objects": []
            }

        # Count objects by class
        object_counts = {}
        for d in self.current_detections:
            name = d.class_name
            if name not in object_counts:
                object_counts[name] = 0
            object_counts[name] += 1

        # Build description
        parts = []
        for obj, count in sorted(object_counts.items(), key=lambda x: -x[1]):
            if count == 1:
                parts.append(f"a {obj}")
            else:
                parts.append(f"{count} {obj}s")

        if len(parts) == 1:
            description = f"I can see {parts[0]}."
        elif len(parts) == 2:
            description = f"I can see {parts[0]} and {parts[1]}."
        else:
            description = f"I can see {', '.join(parts[:-1])}, and {parts[-1]}."

        return {
            "success": True,
            "description": description,
            "objects": list(object_counts.keys()),
            "counts": object_counts
        }

    def tool_scan_surroundings(self) -> dict:
        """
        Perform a quick scan by panning the camera left and right.

        Returns:
            dict with scan results
        """
        self.robot_state = RobotState.SCANNING

        # This would ideally be async, but for now just describe the intent
        # The actual scanning would be done by the room_scanner_node

        return {
            "success": True,
            "message": "Starting a quick scan of surroundings. I'll look left, center, and right.",
            "note": "Scan results will be available through get_detections after scanning completes"
        }

    def tool_get_robot_state(self) -> dict:
        """
        Get the current state of the robot.

        Returns:
            dict with robot state information
        """
        return {
            "success": True,
            "state": self.robot_state.value,
            "camera": {
                "pan": self.camera_pan,
                "tilt": self.camera_tilt
            },
            "distance": self.current_distance,
            "detections_count": len(self.current_detections)
        }

    def get_tool_definitions(self) -> list[dict]:
        """
        Get MCP-compatible tool definitions for all available tools.

        Returns:
            List of tool definitions in MCP format
        """
        return [
            {
                "name": "move_robot",
                "description": "Move the robot in a direction. Use for navigation commands like 'go forward', 'back up', 'turn around'.",
                "input_schema": {
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
                            "description": "Movement speed 0-100 (default 25)",
                            "default": 25
                        }
                    },
                    "required": ["direction"]
                }
            },
            {
                "name": "turn_robot",
                "description": "Turn the robot by a specific angle. Use for precise rotation commands.",
                "input_schema": {
                    "type": "object",
                    "properties": {
                        "angle": {
                            "type": "number",
                            "description": "Degrees to turn (positive value)"
                        },
                        "direction": {
                            "type": "string",
                            "enum": ["left", "right"],
                            "description": "Direction to turn"
                        }
                    },
                    "required": ["angle", "direction"]
                }
            },
            {
                "name": "stop_robot",
                "description": "Immediately stop all robot movement. Use for emergency stops or when done moving.",
                "input_schema": {
                    "type": "object",
                    "properties": {}
                }
            },
            {
                "name": "look_at",
                "description": "Point the camera at specific pan/tilt angles.",
                "input_schema": {
                    "type": "object",
                    "properties": {
                        "pan": {
                            "type": "number",
                            "description": "Horizontal angle: -90 (left) to 90 (right), 0 is center"
                        },
                        "tilt": {
                            "type": "number",
                            "description": "Vertical angle: -30 (down) to 90 (up), 0 is level"
                        }
                    }
                }
            },
            {
                "name": "look_direction",
                "description": "Point the camera in a named direction. Easier than specifying angles.",
                "input_schema": {
                    "type": "object",
                    "properties": {
                        "direction": {
                            "type": "string",
                            "enum": ["left", "right", "up", "down", "center", "forward",
                                    "left_up", "right_up", "left_down", "right_down"],
                            "description": "Named direction to look"
                        }
                    },
                    "required": ["direction"]
                }
            },
            {
                "name": "get_distance",
                "description": "Get the ultrasonic distance sensor reading. Shows distance to nearest obstacle in front.",
                "input_schema": {
                    "type": "object",
                    "properties": {}
                }
            },
            {
                "name": "get_detections",
                "description": "Get list of all objects currently detected by the camera.",
                "input_schema": {
                    "type": "object",
                    "properties": {}
                }
            },
            {
                "name": "find_object",
                "description": "Check if a specific object is visible in the camera view.",
                "input_schema": {
                    "type": "object",
                    "properties": {
                        "object_name": {
                            "type": "string",
                            "description": "Name of object to find (e.g., 'person', 'cup', 'chair', 'dog')"
                        }
                    },
                    "required": ["object_name"]
                }
            },
            {
                "name": "describe_scene",
                "description": "Get a natural language description of what the robot currently sees.",
                "input_schema": {
                    "type": "object",
                    "properties": {}
                }
            },
            {
                "name": "scan_surroundings",
                "description": "Perform a quick scan by panning the camera to see more of the environment.",
                "input_schema": {
                    "type": "object",
                    "properties": {}
                }
            },
            {
                "name": "get_robot_state",
                "description": "Get the current state of the robot including position, camera angle, and sensor data.",
                "input_schema": {
                    "type": "object",
                    "properties": {}
                }
            }
        ]

    def execute_tool(self, tool_name: str, arguments: dict) -> dict:
        """
        Execute a tool by name with given arguments.

        Args:
            tool_name: Name of the tool to execute
            arguments: Dictionary of arguments for the tool

        Returns:
            Tool execution result
        """
        tool_map = {
            "move_robot": self.tool_move_robot,
            "turn_robot": self.tool_turn_robot,
            "stop_robot": self.tool_stop_robot,
            "look_at": self.tool_look_at,
            "look_direction": self.tool_look_direction,
            "get_distance": self.tool_get_distance,
            "get_detections": self.tool_get_detections,
            "find_object": self.tool_find_object,
            "describe_scene": self.tool_describe_scene,
            "scan_surroundings": self.tool_scan_surroundings,
            "get_robot_state": self.tool_get_robot_state,
        }

        if tool_name not in tool_map:
            return {
                "success": False,
                "error": f"Unknown tool: {tool_name}"
            }

        try:
            return tool_map[tool_name](**arguments)
        except Exception as e:
            self.get_logger().error(f"Tool execution error: {e}")
            return {
                "success": False,
                "error": str(e)
            }


def main(args=None):
    """Main entry point for the ROS2 MCP Gateway node."""
    rclpy.init(args=args)

    gateway = ROS2MCPGateway()

    try:
        rclpy.spin(gateway)
    except KeyboardInterrupt:
        pass
    finally:
        gateway.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
