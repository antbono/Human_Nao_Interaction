#!/usr/bin/env python3
from google.cloud import texttospeech
import time
import rclpy
from rclpy.node import Node

from hri_interfaces.srv import TextToSpeech

# string text
# ---
# bool success
# string debug

from sound_play.libsoundplay import SoundClient

from sound_play.msg import SoundRequest

from std_msgs.msg import String


class GTTSService(Node):
    def __init__(self):
        super().__init__("gtts_srv_node")

        # Instantiates a google tts client
        self.client = texttospeech.TextToSpeechClient()

        self.voice = texttospeech.VoiceSelectionParams(
            language_code="IT-IT", name="it-IT-Neural2-C"  # A female, C male
        )

        # Select the type of audio file you want returned
        self.audio_config = texttospeech.AudioConfig(
            audio_encoding=texttospeech.AudioEncoding.OGG_OPUS
        )

        self.sound_handle_b = SoundClient(self, blocking=False)

        self.srv = self.create_service(TextToSpeech, "gtts_service", self.gtts_callback)
        self.get_logger().info("GTTSService Server initialized.")

    def gtts_callback(self, sRequest, sResponse):
        # response.sum = request.a + request.b
        self.get_logger().info("GTTSService Incoming request")

        reqText = sRequest.text.strip()
        if reqText:  # not empty string
            # Set the text input to be synthesized
            synthesis_input = texttospeech.SynthesisInput(text=reqText)

            # Perform the text-to-speech request on the text input with the selected
            # voice parameters and audio file type
            response = self.client.synthesize_speech(
                input=synthesis_input, voice=self.voice, audio_config=self.audio_config
            )

            # The response's audio_content is binary.
            with open("/tmp/output.ogg", "wb") as out:
                # Write the response to the output file.
                out.write(response.audio_content)
                self.get_logger().info('Audio content written to file "output.ogg"')

            self.get_logger().info("Playing output.ogg at full volumeEE.")
            
            self.sound_handle_b.playWave("/tmp/output.ogg")

            self.get_logger().info("Playwave stopped")

            sResponse.success = True

        else:
            sResponse.success = False
            sResponse.debug = "empty text to convert"
            self.get_logger().info("empty text to convert")

        return sResponse


def main():
    rclpy.init()

    gtts_service = GTTSService()

    rclpy.spin(gtts_service)

    rclpy.shutdown()


if __name__ == "__main__":
    main()