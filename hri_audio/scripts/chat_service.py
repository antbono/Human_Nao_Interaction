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


import rclpy
from rclpy.node import Node

from hri_interfaces.srv import Chat

import openai
import os

api_key = os.environ["OPENAI_API_KEY"]

openai.api_key = api_key


class ChatService(Node):

    def __init__(self):
        super().__init__('chat_service_server')
        self.srv = self.create_service(Chat, 'chatGPT_service', self.chat_callback)
        self.chat_messages = [
        {"role": "system", "content": """Sei un robot umanoide chiamato NAO e parli italiano. Ti piacciono i bambini. 
                                        La tua casa è il laboratorio di robotica del DIMES all'Università della Calabria.
                                        Per motivi di sicurezza oggi non ti muovi nell'ambiente."""}
        ]
        self.get_logger().info('ChatService initialized')

    def chat_callback(self, sRequest, sResponse):
        self.get_logger().info("Incoming request: " + sRequest.text)
        self.chat_messages.append({"role": "user", "content": sRequest.text})
        reply = self.get_response(messages=self.chat_messages)
        reply_text = reply['content'];
        self.get_logger().info("Reply: " + reply_text)
        self.chat_messages.append(reply)
        sResponse.debug = reply_text
        return sResponse    
        
    def get_response(self, messages:list):
        response = openai.ChatCompletion.create(
            model = "gpt-3.5-turbo",
            messages = messages,
            temperature = 1.0 # 0.0 - 2.0
        )
        return response.choices[0].message


def main(args=None):
    rclpy.init(args=args)

    minimal_service = ChatService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()