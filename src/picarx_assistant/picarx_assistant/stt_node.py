#!/usr/bin/env python3
"""
Speech-to-Text Node - Converts audio to text.

Supports:
- Local: faster-whisper (optimized Whisper for CPU/GPU)
- Cloud: OpenAI Whisper API
"""

import os
import tempfile
import wave
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Bool

# Try to import STT backends
try:
    from faster_whisper import WhisperModel
    FASTER_WHISPER_AVAILABLE = True
except ImportError:
    FASTER_WHISPER_AVAILABLE = False

try:
    import openai
    OPENAI_AVAILABLE = True
except ImportError:
    OPENAI_AVAILABLE = False


class STTNode(Node):
    """
    Speech-to-Text node using Whisper.

    Converts audio chunks to text transcriptions.
    """

    def __init__(self):
        super().__init__('stt_node')

        # Declare parameters
        self.declare_parameter('backend', 'whisper_local')  # whisper_local, whisper_api
        self.declare_parameter('model', 'base.en')  # tiny, base, small, medium, large
        self.declare_parameter('language', 'en')
        self.declare_parameter('device', 'cpu')  # cpu, cuda
        self.declare_parameter('compute_type', 'int8')  # int8, float16, float32
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('openai_api_key', '')

        # Get parameters
        self.backend = self.get_parameter('backend').value
        self.model_name = self.get_parameter('model').value
        self.language = self.get_parameter('language').value
        self.device = self.get_parameter('device').value
        self.compute_type = self.get_parameter('compute_type').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.openai_key = self.get_parameter('openai_api_key').value or os.environ.get('OPENAI_API_KEY', '')

        # Initialize model
        self.model = None
        self.openai_client = None

        if self.backend == 'whisper_local' and FASTER_WHISPER_AVAILABLE:
            self.get_logger().info(f'Loading Whisper model: {self.model_name} on {self.device}')
            try:
                self.model = WhisperModel(
                    self.model_name,
                    device=self.device,
                    compute_type=self.compute_type
                )
                self.get_logger().info('Whisper model loaded successfully')
            except Exception as e:
                self.get_logger().error(f'Failed to load Whisper model: {e}')

        elif self.backend == 'whisper_api' and OPENAI_AVAILABLE and self.openai_key:
            self.openai_client = openai.OpenAI(api_key=self.openai_key)
            self.get_logger().info('Using OpenAI Whisper API')

        else:
            self.get_logger().warn('No STT backend available - transcription disabled')

        # QoS
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Publishers
        self.transcription_pub = self.create_publisher(
            String,
            '/voice/transcription',
            qos
        )
        self.processing_pub = self.create_publisher(
            Bool,
            '/voice/stt_processing',
            qos
        )

        # Subscribers
        self.audio_sub = self.create_subscription(
            String,
            '/voice/audio_chunk',
            self._audio_callback,
            qos
        )

        self.get_logger().info('STT node initialized')

    def _audio_callback(self, msg: String):
        """Process incoming audio chunk."""
        try:
            # Decode hex-encoded audio
            audio_data = bytes.fromhex(msg.data)

            self._publish_processing(True)
            transcription = self._transcribe(audio_data)
            self._publish_processing(False)

            if transcription:
                self.get_logger().info(f'Transcription: "{transcription}"')
                trans_msg = String()
                trans_msg.data = transcription
                self.transcription_pub.publish(trans_msg)

        except Exception as e:
            self.get_logger().error(f'Transcription error: {e}')
            self._publish_processing(False)

    def _transcribe(self, audio_data: bytes) -> Optional[str]:
        """Transcribe audio data to text."""
        if self.model:
            return self._transcribe_local(audio_data)
        elif self.openai_client:
            return self._transcribe_api(audio_data)
        else:
            return None

    def _transcribe_local(self, audio_data: bytes) -> Optional[str]:
        """Transcribe using local Whisper model."""
        # Save to temporary WAV file
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
            temp_path = f.name
            with wave.open(f, 'wb') as wav:
                wav.setnchannels(1)
                wav.setsampwidth(2)  # 16-bit
                wav.setframerate(self.sample_rate)
                wav.writeframes(audio_data)

        try:
            segments, info = self.model.transcribe(
                temp_path,
                language=self.language if self.language != 'auto' else None,
                beam_size=5,
                vad_filter=True
            )

            text = ' '.join(segment.text.strip() for segment in segments)
            return text.strip() if text else None

        finally:
            os.unlink(temp_path)

    def _transcribe_api(self, audio_data: bytes) -> Optional[str]:
        """Transcribe using OpenAI Whisper API."""
        # Save to temporary file
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as f:
            temp_path = f.name
            with wave.open(f, 'wb') as wav:
                wav.setnchannels(1)
                wav.setsampwidth(2)
                wav.setframerate(self.sample_rate)
                wav.writeframes(audio_data)

        try:
            with open(temp_path, 'rb') as audio_file:
                transcript = self.openai_client.audio.transcriptions.create(
                    model="whisper-1",
                    file=audio_file,
                    language=self.language if self.language != 'auto' else None
                )
            return transcript.text.strip() if transcript.text else None

        finally:
            os.unlink(temp_path)

    def _publish_processing(self, processing: bool):
        """Publish processing state."""
        msg = Bool()
        msg.data = processing
        self.processing_pub.publish(msg)


def main(args=None):
    """Main entry point for STT node."""
    rclpy.init(args=args)

    node = STTNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
