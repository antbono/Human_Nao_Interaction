#!/usr/bin/env python3

# Copyright 2024 Antonio Bono
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from google.cloud import speech
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from hri_audio.microphone_stream import *
from std_srvs.srv import SetBool

# bool data # e.g. for hardware enabling / disabling
# ---
# bool success   # indicate successful run of triggered serviceyapf
# string message # informational, e.g. for error messages

# Audio recording parameters
RATE = 16000
CHUNK = int(RATE / 10)  # 100ms

class GSTTService(Node):
    def __init__(self):
        super().__init__("gstt_srv_node")
        language_code = "it-IT"
        self.client = speech.SpeechClient()
        self.config = speech.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=RATE,
            language_code=language_code,
        )

        self.streaming_config = speech.StreamingRecognitionConfig(
            config=self.config,
            single_utterance=True
            #interim_results=True
            # config=config, enable_voice_activity_events=True, voice_activity_timeout=True, speech_end_timeout=4
        )
        self.srv = self.create_service(SetBool, "gstt_service", self.gstt_callback)
        
    
    def gstt_callback(self, sRequest, sResponse):
        # response.sum = request.a + request.b
        self.get_logger().info('GSTTService Incoming request')
        if sRequest.data == True:

            with MicrophoneStream(RATE, CHUNK) as stream:
                audio_generator = stream.generator()
                requests = (
                    speech.StreamingRecognizeRequest(audio_content=content)
                    for content in audio_generator
                )
                responses = self.client.streaming_recognize(self.streaming_config, requests)
                # Now, put the transcription responses to use.
                #listen_print_loop(responses)
                sResponse.success = True
                sResponse.message = self.__retrieve_text(responses)
                return sResponse
        else:
            sResponse.data = False
            return sResponse
            

    def __retrieve_text(self, responses):
            """Iterates through server responses and prints them.

            The responses passed is a generator that will block until a response
            is provided by the server.

            Each response may contain multiple results, and each result may contain
            multiple alternatives; for details, see https://goo.gl/tjCPAU.  Here we
            print only the transcription for the top alternative of the top result.

            In this case, responses are provided for interim results as well. If the
            response is an interim one, print a line feed at the end of it, to allow
            the next result to overwrite it, until the response is a final one. For the
            final one, print a newline to preserve the finalized transcription.
            """
            num_chars_printed = 0
            for response in responses:
                if not response.results:
                    continue

                # The `results` list is consecutive. For streaming, we only care about
                # the first result being considered, since once it's `is_final`, it
                # moves on to considering the next utterance.
                result = response.results[0]
                if not result.alternatives:
                    continue

                # Display the transcription of the top alternative.
                transcript = result.alternatives[0].transcript

                # Display interim results, but with a carriage return at the end of the
                # line, so subsequent lines will overwrite them.
                #
                # If the previous result was longer than this one, we need to print
                # some extra spaces to overwrite the previous result
                overwrite_chars = " " * (num_chars_printed - len(transcript))

                if result.is_final:
                    return transcript    
    



def main():
    rclpy.init()

    gstt_service = GSTTService()

    rclpy.spin(gstt_service)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
