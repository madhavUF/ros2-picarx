#!/usr/bin/env python3
"""
Text-to-Speech Node - Converts text to speech audio.

Supports:
- Local: piper-tts (fast, runs on Pi)
- Cloud: OpenAI TTS API
- Cloud: ElevenLabs API
"""

import os
import subprocess
import tempfile
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Bool

# Try to import TTS backends
try:
    import openai
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False

# Piper is typically run as a subprocess, no import needed


class TTSNode(Node):
    """
    Text-to-Speech node.

    Converts text to speech and plays it through speakers.
    """

    def __init__(self):
        super().__init__('tts_node')

        # Declare parameters
        self.declare_parameter('backend', 'piper')  # piper, openai, espeak
        self.declare_parameter('voice', 'en_US-lessac-medium')  # Piper voice
        self.declare_parameter('openai_voice', 'alloy')  # OpenAI voice
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('openai_api_key', '')
        self.declare_parameter('piper_path', 'piper')  # Path to piper executable
        self.declare_parameter('output_device', 'default')

        # Get parameters
        self.backend = self.get_parameter('backend').value
        self.piper_voice = self.get_parameter('voice').value
        self.openai_voice = self.get_parameter('openai_voice').value
        self.speed = self.get_parameter('speed').value
        self.openai_key = self.get_parameter('openai_api_key').value or os.environ.get('OPENAI_API_KEY', '')
        self.piper_path = self.get_parameter('piper_path').value
        self.output_device = self.get_parameter('output_device').value

        # Initialize OpenAI client
        self.openai_client = None
        if self.backend == 'openai' and OPENAI_AVAILABLE and self.openai_key:
            self.openai_client = openai.OpenAI(api_key=self.openai_key)
            self.get_logger().info('Using OpenAI TTS')
        elif self.backend == 'piper':
            self.get_logger().info(f'Using Piper TTS with voice: {self.piper_voice}')
        elif self.backend == 'espeak':
            self.get_logger().info('Using eSpeak TTS (fallback)')

        # State
        self.is_speaking = False
        self.speech_queue = []
        self.speech_lock = threading.Lock()

        # QoS
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Publishers
        self.speaking_pub = self.create_publisher(
            Bool,
            '/voice/is_speaking',
            qos
        )

        # Subscribers
        self.text_sub = self.create_subscription(
            String,
            '/voice/response_text',
            self._text_callback,
            qos
        )

        # Speech processing thread
        self.running = True
        self.speech_thread = threading.Thread(target=self._speech_loop)
        self.speech_thread.daemon = True
        self.speech_thread.start()

        self.get_logger().info('TTS node initialized')

    def _text_callback(self, msg: String):
        """Queue text for speech synthesis."""
        text = msg.data.strip()
        if text:
            with self.speech_lock:
                self.speech_queue.append(text)
            self.get_logger().info(f'Queued for TTS: "{text[:50]}..."')

    def _speech_loop(self):
        """Background thread for speech synthesis."""
        while self.running:
            text = None
            with self.speech_lock:
                if self.speech_queue:
                    text = self.speech_queue.pop(0)

            if text:
                self._speak(text)
            else:
                import time
                time.sleep(0.1)

    def _speak(self, text: str):
        """Synthesize and play speech."""
        self._publish_speaking(True)

        try:
            if self.backend == 'openai' and self.openai_client:
                self._speak_openai(text)
            elif self.backend == 'piper':
                self._speak_piper(text)
            elif self.backend == 'espeak':
                self._speak_espeak(text)
            else:
                self.get_logger().warn(f'Unknown TTS backend: {self.backend}')
                self._speak_espeak(text)  # Fallback

        except Exception as e:
            self.get_logger().error(f'TTS error: {e}')

        finally:
            self._publish_speaking(False)

    def _speak_piper(self, text: str):
        """Use Piper TTS (local, fast)."""
        try:
            # Piper command: echo "text" | piper --model voice.onnx --output_file out.wav
            # Then play with aplay/paplay

            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
                temp_path = f.name

            # Run piper
            cmd = f'echo "{text}" | {self.piper_path} --model {self.piper_voice} --output_file {temp_path}'
            result = subprocess.run(cmd, shell=True, capture_output=True, timeout=30)

            if result.returncode == 0 and os.path.exists(temp_path):
                # Play audio
                self._play_audio(temp_path)
            else:
                self.get_logger().warn(f'Piper failed: {result.stderr.decode()}')
                # Fallback to espeak
                self._speak_espeak(text)

            # Cleanup
            if os.path.exists(temp_path):
                os.unlink(temp_path)

        except FileNotFoundError:
            self.get_logger().warn('Piper not found, falling back to espeak')
            self._speak_espeak(text)
        except Exception as e:
            self.get_logger().error(f'Piper error: {e}')
            self._speak_espeak(text)

    def _speak_openai(self, text: str):
        """Use OpenAI TTS API."""
        try:
            response = self.openai_client.audio.speech.create(
                model="tts-1",
                voice=self.openai_voice,
                input=text,
                speed=self.speed
            )

            with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as f:
                temp_path = f.name
                response.stream_to_file(temp_path)

            self._play_audio(temp_path)
            os.unlink(temp_path)

        except Exception as e:
            self.get_logger().error(f'OpenAI TTS error: {e}')
            self._speak_espeak(text)

    def _speak_espeak(self, text: str):
        """Use eSpeak (always available fallback)."""
        try:
            # Escape quotes in text
            safe_text = text.replace('"', '\\"').replace("'", "\\'")
            cmd = f'espeak -v en -s {int(150 * self.speed)} "{safe_text}"'
            subprocess.run(cmd, shell=True, timeout=60)
        except Exception as e:
            self.get_logger().error(f'eSpeak error: {e}')

    def _play_audio(self, filepath: str):
        """Play an audio file."""
        try:
            # Try different players
            players = [
                f'aplay -D {self.output_device} {filepath}' if self.output_device != 'default' else f'aplay {filepath}',
                f'paplay {filepath}',
                f'mpv --no-video {filepath}',
                f'ffplay -nodisp -autoexit {filepath}',
            ]

            for player_cmd in players:
                try:
                    result = subprocess.run(player_cmd, shell=True, capture_output=True, timeout=60)
                    if result.returncode == 0:
                        return
                except (subprocess.TimeoutExpired, FileNotFoundError):
                    continue

            self.get_logger().warn('No audio player found')

        except Exception as e:
            self.get_logger().error(f'Audio playback error: {e}')

    def _publish_speaking(self, speaking: bool):
        """Publish speaking state."""
        self.is_speaking = speaking
        msg = Bool()
        msg.data = speaking
        self.speaking_pub.publish(msg)

    def destroy_node(self):
        """Clean up resources."""
        self.running = False
        if self.speech_thread.is_alive():
            self.speech_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    """Main entry point for TTS node."""
    rclpy.init(args=args)

    node = TTSNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
