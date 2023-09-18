#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node

from std_srvs.srv import SetBool
# bool data # e.g. for hardware enabling / disabling
# ---
# bool success   # indicate successful run of triggered service
# string message # informational, e.g. for error messages

from hri_interfaces.srv import TextToSpeech
# string text
# ---
# bool success
# string debug

import os
import openai

api_key = os.environ["OPENAI_API_KEY"]

openai.api_key = api_key



class ChatNode(Node):

    def __init__(self):
        super().__init__('chat_node')

        self.gstt_client = self.create_client(SetBool, 'gstt_service')
        while not self.gstt_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('gstt_service not available, waiting again...')
        self.gstt_req = SetBool.Request()

        self.gtts_client = self.create_client(TextToSpeech, 'gtts_service')
        while not self.gtts_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('gtts_service not available, waiting again...')
        self.gtts_req = TextToSpeech.Request()

    def send_gstt_req(self, cmd):
        self.gstt_req.data = cmd
        self.future = self.gstt_client.call_async(self.gstt_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def send_gtts_req(self, text):
        self.gtts_req.text = text
        self.future = self.gtts_client.call_async(self.gtts_req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def get_response(self, messages:list):
        response = openai.ChatCompletion.create(
            model = "gpt-3.5-turbo",
            messages=messages,
            temperature = 1.0 # 0.0 - 2.0
        )
        return response.choices[0].message



def main(args=None):
    rclpy.init(args=args)

    chat = ChatNode()
    start = True;
    messages = [
        {"role": "system", "content": "Sei un robot umanoide chiamato NAO e parli italiano. Ti piacciono i bambini. La tua casa è il laboratorio di robotica del DIMES all'Università della Calabria."}
    ]

    try:
        while True:
            gstt_resp = chat.send_gstt_req(start)
            print(gstt_resp.message)
            chat.get_logger().info('stt request complete')

            messages.append({"role": "user", "content": gstt_resp.message})
            new_message = chat.get_response(messages=messages)

            gtts_resp = chat.send_gtts_req(new_message['content'])
            print(gtts_resp.debug)
            chat.get_logger().info(' tts Request complete')

            messages.append(new_message)

            user_input = input("Premi dopo aver sentito la risposta")

    except KeyboardInterrupt:
        print("A presto!")

    chat.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()