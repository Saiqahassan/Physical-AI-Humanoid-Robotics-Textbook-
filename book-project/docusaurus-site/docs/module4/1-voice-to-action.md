# Chapter 1: Voice-to-Action: Speech Interfaces for Humanoid Robots

This chapter introduces the foundational components of a Voice-Language-Action (VLA) system: converting spoken language into robotic actions. We will build a simple but powerful pipeline using ROS 2 that allows a user to control a simulated humanoid robot with their voice.

## Core Concepts

- **Speech-to-Text (STT)**: The process of converting audio signals into written text. We will use OpenAI's Whisper model, a state-of-the-art STT system, for its high accuracy and ease of use.
- **ROS 2 Nodes**: The fundamental building blocks of a ROS 2 application. We will create two primary nodes: one for speech recognition and one for robot control.
- **ROS 2 Topics**: The communication channels that allow nodes to exchange data. Our nodes will use topics to pass transcribed text and robot commands.

## The Voice-to-Action Pipeline

Our pipeline will consist of the following steps:

1.  A ROS 2 node captures audio (for simplicity, we assume an external node is publishing audio to a topic).
2.  The `speech_to_text_node` subscribes to the audio topic, receives the audio data, and sends it to the Whisper API for transcription.
3.  The Whisper API returns the transcribed text.
4.  The `speech_to_text_node` publishes the transcribed text to a `/transcribed_text` topic.
5.  The `robot_controller_node` subscribes to the `/transcribed_text` topic.
6.  When a command is received, the `robot_controller_node` matches it against a set of predefined actions (e.g., "wave", "walk").
7.  If a match is found, the node publishes the corresponding command to the robot's joint controllers.

```plantuml
@startuml
skinparam handwritten true

actor "User" as user
participant "Audio Source Node" as audio_node
participant "SpeechToTextNode" as stt_node
participant "OpenAI Whisper API" as whisper_api
participant "RobotControllerNode" as rc_node
participant "Robot Joint Controllers" as robot_joints

user -> audio_node : Spoken Command
audio_node --(0)-> stt_node : Audio Data (/audio topic)
stt_node -> whisper_api : Audio for Transcription
whisper_api --> stt_node : Transcribed Text
stt_node --(0)-> rc_node : Transcribed Text (/transcribed_text topic)
rc_node -> robot_joints : Joint Commands
robot_joints -> rc_node : (Feedback)
rc_node -> user : (Visual Confirmation in Sim)

@enduml
```

## Implementation

### Speech-to-Text Node

Here is the code for our `speech_to_text_node.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray
import openai
import os
import numpy as np
import wave

class SpeechToTextNode(Node):
    def __init__(self):
        super().__init__('speech_to_text_node')
        self.subscription = self.create_subscription(
            Int16MultiArray,
            'audio',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(String, 'transcribed_text', 10)
        openai.api_key = os.getenv("OPENAI_API_KEY")
        self.get_logger().info('Speech to text node started')

    def listener_callback(self, msg):
        self.get_logger().info('Received audio data')
        
        # Convert to numpy array
        audio_data = np.array(msg.data, dtype=np.int16)

        # Save as a temporary wav file
        with wave.open("temp_audio.wav", "w") as f:
            f.setnchannels(1)
            f.setsampwidth(2)
            f.setframerate(16000) # Assuming 16kHz sample rate
            f.writeframes(audio_data.tobytes())

        # Transcribe
        try:
            with open("temp_audio.wav", "rb") as audio_file:
                transcript = openai.Audio.transcribe("whisper-1", audio_file)
            
            text = transcript['text']
            self.get_logger().info(f'Transcription: "{text}"')

            # Publish the text
            text_msg = String()
            text_msg.data = text
            self.publisher.publish(text_msg)

        except Exception as e:
            self.get_logger().error(f"Error during transcription: {e}")

# ... (main function)
```

### Robot Controller Node

This node listens for transcribed commands and executes them.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
import time

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')
        self.subscription = self.create_subscription(
            String,
            'transcribed_text',
            self.listener_callback,
            10)
        
        # Publishers for joint positions
        self.joint_position_publisher = self.create_publisher(
            Float64MultiArray, 
            '/humanoid_joint_controller/commands', 
            10
        )

        self.get_logger().info('Robot controller node started')

    def listener_callback(self, msg):
        command = msg.data.lower()
        self.get_logger().info(f'Received command: "{command}"')

        if "wave" in command:
            self.wave_hello()
        elif "walk" in command:
            self.walk_forward()

    # ... (wave_hello and walk_forward methods)
```

By the end of this chapter, you will have a functional voice-controlled robot in simulation, setting the stage for more advanced AI-driven behaviors in the chapters to come.