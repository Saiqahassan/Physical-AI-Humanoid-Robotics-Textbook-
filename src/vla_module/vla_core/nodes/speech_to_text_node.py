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


def main(args=None):
    rclpy.init(args=args)
    speech_to_text_node = SpeechToTextNode()
    rclpy.spin(speech_to_text_node)
    speech_to_text_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
