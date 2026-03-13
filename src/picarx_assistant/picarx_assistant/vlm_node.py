#!/usr/bin/env python3
"""
Vision-Language Model Node - Scene understanding and visual Q&A.

This node provides natural language descriptions of what the robot sees,
going beyond simple object detection to scene understanding.
"""

import os
import base64
import tempfile
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from sensor_msgs.msg import Image

# Try to import VLM backends
try:
    import anthropic
    ANTHROPIC_AVAILABLE = True
except ImportError:
    ANTHROPIC_AVAILABLE = False

try:
    import cv2
    from cv_bridge import CvBridge
    CV_AVAILABLE = True
except ImportError:
    CV_AVAILABLE = False


class VLMNode(Node):
    """
    Vision-Language Model node for scene understanding.

    Uses Claude Vision or other VLMs to describe scenes and answer
    questions about what the robot sees.
    """

    def __init__(self):
        super().__init__('vlm_node')

        # Declare parameters
        self.declare_parameter('backend', 'claude')  # claude, openai, local
        self.declare_parameter('model', 'claude-sonnet-4-20250514')
        self.declare_parameter('api_key', '')
        self.declare_parameter('max_tokens', 300)
        self.declare_parameter('image_topic', '/vision/image_raw')

        # Get parameters
        self.backend = self.get_parameter('backend').value
        self.model = self.get_parameter('model').value
        self.api_key = self.get_parameter('api_key').value or os.environ.get('ANTHROPIC_API_KEY', '')
        self.max_tokens = self.get_parameter('max_tokens').value
        self.image_topic = self.get_parameter('image_topic').value

        # Initialize client
        self.client = None
        if self.backend == 'claude' and ANTHROPIC_AVAILABLE and self.api_key:
            self.client = anthropic.Anthropic(api_key=self.api_key)
            self.get_logger().info('Using Claude Vision')
        else:
            self.get_logger().warn('No VLM backend available')

        # Image handling
        self.cv_bridge = CvBridge() if CV_AVAILABLE else None
        self.latest_image = None
        self.latest_image_base64 = None

        # QoS
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self._image_callback,
            qos
        )

        # Query subscriber (for external requests)
        self.query_sub = self.create_subscription(
            String,
            '/vlm/query',
            self._query_callback,
            qos_reliable
        )

        # Publishers
        self.description_pub = self.create_publisher(
            String,
            '/vlm/description',
            qos_reliable
        )
        self.answer_pub = self.create_publisher(
            String,
            '/vlm/answer',
            qos_reliable
        )

        self.get_logger().info('VLM node initialized')

    def _image_callback(self, msg: Image):
        """Store latest image for processing."""
        if not self.cv_bridge:
            return

        try:
            # Convert ROS Image to OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = cv_image

            # Encode to base64 for API
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            self.latest_image_base64 = base64.b64encode(buffer).decode('utf-8')

        except Exception as e:
            self.get_logger().error(f'Image conversion error: {e}')

    def _query_callback(self, msg: String):
        """Handle incoming VLM queries."""
        query = msg.data.strip()
        if not query:
            return

        self.get_logger().info(f'VLM query: "{query}"')

        if query.lower() == 'describe':
            result = self.describe_scene()
        else:
            result = self.answer_question(query)

        if result:
            answer_msg = String()
            answer_msg.data = result
            self.answer_pub.publish(answer_msg)

    def describe_scene(self) -> Optional[str]:
        """Generate a natural language description of the current scene."""
        if not self.latest_image_base64:
            return "I don't have a camera image to describe."

        if not self.client:
            return "Vision-language model not available."

        try:
            prompt = """Describe what you see in this image from a robot's camera.
Be concise (2-3 sentences). Focus on:
- Main objects and their approximate positions
- People if present
- General environment (indoor/outdoor, room type)
- Any notable features or obstacles

Speak in first person as if you are the robot ("I can see...")."""

            response = self.client.messages.create(
                model=self.model,
                max_tokens=self.max_tokens,
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "image",
                                "source": {
                                    "type": "base64",
                                    "media_type": "image/jpeg",
                                    "data": self.latest_image_base64
                                }
                            },
                            {
                                "type": "text",
                                "text": prompt
                            }
                        ]
                    }
                ]
            )

            description = response.content[0].text
            self.get_logger().info(f'Scene description: {description[:100]}...')

            # Publish description
            desc_msg = String()
            desc_msg.data = description
            self.description_pub.publish(desc_msg)

            return description

        except Exception as e:
            self.get_logger().error(f'VLM error: {e}')
            return f"I couldn't analyze the image: {str(e)}"

    def answer_question(self, question: str) -> Optional[str]:
        """Answer a question about the current camera view."""
        if not self.latest_image_base64:
            return "I don't have a camera image to analyze."

        if not self.client:
            return "Vision-language model not available."

        try:
            prompt = f"""You are a helpful robot assistant. Look at this image from your camera and answer the question.
Keep your answer concise and natural, as if speaking aloud.

Question: {question}"""

            response = self.client.messages.create(
                model=self.model,
                max_tokens=self.max_tokens,
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {
                                "type": "image",
                                "source": {
                                    "type": "base64",
                                    "media_type": "image/jpeg",
                                    "data": self.latest_image_base64
                                }
                            },
                            {
                                "type": "text",
                                "text": prompt
                            }
                        ]
                    }
                ]
            )

            answer = response.content[0].text
            self.get_logger().info(f'VLM answer: {answer[:100]}...')
            return answer

        except Exception as e:
            self.get_logger().error(f'VLM error: {e}')
            return f"I couldn't analyze the image: {str(e)}"

    def get_image_base64(self) -> Optional[str]:
        """Get the latest image as base64 for external use."""
        return self.latest_image_base64


def main(args=None):
    """Main entry point for VLM node."""
    rclpy.init(args=args)

    node = VLMNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
