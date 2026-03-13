from setuptools import setup, find_packages

package_name = 'picarx_assistant'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PiCar-X Team',
    maintainer_email='user@example.com',
    description='Voice-controlled AI assistant for PiCar-X robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'voice_input_node = picarx_assistant.voice_input_node:main',
            'stt_node = picarx_assistant.stt_node:main',
            'tts_node = picarx_assistant.tts_node:main',
            'conversation_node = picarx_assistant.conversation_node:main',
            'ros2_mcp_server = picarx_assistant.ros2_mcp_server:main',
            'vlm_node = picarx_assistant.vlm_node:main',
            'assistant_bridge = picarx_assistant.assistant_bridge:main',
        ],
    },
)
