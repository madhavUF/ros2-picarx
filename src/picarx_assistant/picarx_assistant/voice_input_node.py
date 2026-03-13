#!/usr/bin/env python3
"""
Voice Input Node - Captures audio from microphone with voice activity detection.

This node:
1. Captures audio from microphone
2. Performs voice activity detection (VAD)
3. Buffers speech segments
4. Publishes audio chunks for STT processing
"""

import struct
import threading
import queue
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, Bool, Header

# Try to import audio libraries
try:
    import pyaudio
    PYAUDIO_AVAILABLE = True
except ImportError:
    PYAUDIO_AVAILABLE = False

try:
    import webrtcvad
    VAD_AVAILABLE = True
except ImportError:
    VAD_AVAILABLE = False


class VoiceInputNode(Node):
    """
    Captures audio from microphone and detects speech.

    Uses WebRTC VAD for voice activity detection and publishes
    audio chunks when speech is detected.
    """

    def __init__(self):
        super().__init__('voice_input_node')

        # Declare parameters
        self.declare_parameter('device_index', -1)  # -1 for default
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('channels', 1)
        self.declare_parameter('chunk_duration_ms', 30)  # VAD works with 10, 20, or 30ms
        self.declare_parameter('vad_aggressiveness', 2)  # 0-3, higher = more aggressive
        self.declare_parameter('min_speech_duration', 0.3)  # seconds
        self.declare_parameter('silence_duration', 0.8)  # seconds of silence to end utterance
        self.declare_parameter('wake_word', '')  # Optional wake word
        self.declare_parameter('always_listening', True)

        # Get parameters
        self.device_index = self.get_parameter('device_index').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.channels = self.get_parameter('channels').value
        self.chunk_duration = self.get_parameter('chunk_duration_ms').value
        self.vad_level = self.get_parameter('vad_aggressiveness').value
        self.min_speech = self.get_parameter('min_speech_duration').value
        self.silence_duration = self.get_parameter('silence_duration').value
        self.wake_word = self.get_parameter('wake_word').value
        self.always_listening = self.get_parameter('always_listening').value

        # Calculate chunk size
        self.chunk_size = int(self.sample_rate * self.chunk_duration / 1000)

        # State
        self.is_recording = False
        self.is_speaking = False
        self.speech_buffer = []
        self.silence_chunks = 0
        self.speech_chunks = 0
        self.audio_queue = queue.Queue()

        # Initialize VAD
        self.vad = None
        if VAD_AVAILABLE:
            self.vad = webrtcvad.Vad(self.vad_level)
            self.get_logger().info(f'VAD initialized with aggressiveness {self.vad_level}')
        else:
            self.get_logger().warn('webrtcvad not available - using energy-based detection')

        # Initialize PyAudio
        self.pa = None
        self.stream = None
        if PYAUDIO_AVAILABLE:
            self.pa = pyaudio.PyAudio()
            self._log_audio_devices()
        else:
            self.get_logger().error('PyAudio not available - voice input disabled')

        # QoS
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Publishers
        self.audio_pub = self.create_publisher(
            String,  # Base64 encoded audio or raw bytes as hex
            '/voice/audio_chunk',
            qos
        )
        self.speech_detected_pub = self.create_publisher(
            Bool,
            '/voice/speech_detected',
            qos
        )
        self.listening_pub = self.create_publisher(
            Bool,
            '/voice/is_listening',
            qos
        )

        # Subscribers
        self.speaking_sub = self.create_subscription(
            Bool,
            '/voice/is_speaking',
            self._speaking_callback,
            qos
        )

        # Start audio capture thread
        self.running = True
        self.audio_thread = None
        if PYAUDIO_AVAILABLE:
            self.audio_thread = threading.Thread(target=self._audio_capture_loop)
            self.audio_thread.daemon = True
            self.audio_thread.start()

        # Processing timer
        self.process_timer = self.create_timer(0.01, self._process_audio)

        self._publish_listening_state(True)
        self.get_logger().info('Voice input node initialized')

    def _log_audio_devices(self):
        """Log available audio devices."""
        self.get_logger().info('Available audio input devices:')
        for i in range(self.pa.get_device_count()):
            info = self.pa.get_device_info_by_index(i)
            if info['maxInputChannels'] > 0:
                self.get_logger().info(f'  [{i}] {info["name"]}')

    def _speaking_callback(self, msg: Bool):
        """Pause listening while TTS is speaking to avoid feedback."""
        self.is_speaking = msg.data
        if self.is_speaking:
            self.get_logger().debug('Pausing voice input (TTS speaking)')
        else:
            self.get_logger().debug('Resuming voice input')

    def _publish_listening_state(self, listening: bool):
        """Publish whether we're actively listening."""
        msg = Bool()
        msg.data = listening
        self.listening_pub.publish(msg)

    def _audio_capture_loop(self):
        """Background thread for audio capture."""
        try:
            device = self.device_index if self.device_index >= 0 else None
            self.stream = self.pa.open(
                format=pyaudio.paInt16,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                input_device_index=device,
                frames_per_buffer=self.chunk_size
            )
            self.get_logger().info('Audio stream opened')

            while self.running:
                try:
                    audio_data = self.stream.read(self.chunk_size, exception_on_overflow=False)
                    if not self.is_speaking:  # Don't capture while speaking
                        self.audio_queue.put(audio_data)
                except Exception as e:
                    self.get_logger().warn(f'Audio read error: {e}')

        except Exception as e:
            self.get_logger().error(f'Failed to open audio stream: {e}')

    def _process_audio(self):
        """Process audio chunks from the queue."""
        try:
            while not self.audio_queue.empty():
                audio_data = self.audio_queue.get_nowait()
                self._handle_audio_chunk(audio_data)
        except queue.Empty:
            pass

    def _handle_audio_chunk(self, audio_data: bytes):
        """Process a single audio chunk with VAD."""
        is_speech = self._detect_speech(audio_data)

        if is_speech:
            self.speech_chunks += 1
            self.silence_chunks = 0

            if not self.is_recording:
                # Start recording
                min_chunks = int(self.min_speech * 1000 / self.chunk_duration)
                if self.speech_chunks >= min_chunks:
                    self.is_recording = True
                    self.get_logger().info('Speech started')
                    self._publish_speech_detected(True)

            if self.is_recording:
                self.speech_buffer.append(audio_data)

        else:  # Silence
            self.speech_chunks = 0
            self.silence_chunks += 1

            if self.is_recording:
                self.speech_buffer.append(audio_data)

                # Check for end of utterance
                max_silence_chunks = int(self.silence_duration * 1000 / self.chunk_duration)
                if self.silence_chunks >= max_silence_chunks:
                    self._end_utterance()

    def _detect_speech(self, audio_data: bytes) -> bool:
        """Detect if audio chunk contains speech."""
        if self.vad:
            try:
                return self.vad.is_speech(audio_data, self.sample_rate)
            except Exception:
                return self._energy_based_detection(audio_data)
        else:
            return self._energy_based_detection(audio_data)

    def _energy_based_detection(self, audio_data: bytes, threshold: int = 500) -> bool:
        """Simple energy-based speech detection fallback."""
        # Convert bytes to samples
        samples = struct.unpack(f'{len(audio_data)//2}h', audio_data)
        energy = sum(abs(s) for s in samples) / len(samples)
        return energy > threshold

    def _end_utterance(self):
        """Finalize and publish the recorded utterance."""
        if not self.speech_buffer:
            return

        self.get_logger().info(f'Speech ended - {len(self.speech_buffer)} chunks')

        # Combine all audio chunks
        full_audio = b''.join(self.speech_buffer)

        # Publish as hex-encoded string (could also use base64)
        msg = String()
        msg.data = full_audio.hex()
        self.audio_pub.publish(msg)

        # Reset state
        self.speech_buffer = []
        self.is_recording = False
        self.silence_chunks = 0
        self._publish_speech_detected(False)

    def _publish_speech_detected(self, detected: bool):
        """Publish speech detection state."""
        msg = Bool()
        msg.data = detected
        self.speech_detected_pub.publish(msg)

    def destroy_node(self):
        """Clean up resources."""
        self.running = False
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        if self.pa:
            self.pa.terminate()
        super().destroy_node()


def main(args=None):
    """Main entry point for voice input node."""
    rclpy.init(args=args)

    node = VoiceInputNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
