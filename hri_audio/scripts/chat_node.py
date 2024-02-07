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

    def get_response4(self, messages:list):
        response = openai.ChatCompletion.create(
            #model = "gpt-3.5-turbo",
            model = "gpt-4",
            messages=messages,
            temperature = 1.0 # 0.0 - 2.0
        )
        return response.choices[0].message


def main(args=None):
    rclpy.init(args=args)

    print("main")

    chat = ChatNode()
    start = True;
    messages_nao = [
        {"role": "system", "content": "Sei un robot umanoide chiamato NAO. e parli preferibilmente\
         italiano ma conosci e parli inglese anche. Alcune tue caratteristiche: \
         - Parli preferibilmente italiano ma conosci anche altre lingue. \
         - La tua casa è il laboratorio di robotica del DIMES all'Università della Calabria. Il tuo sviluppatore principale è Antonio Bono. \
         - Puoi riprodurre solo alcune canzoni che hai salvato nella tua memoria ma non canzoni qualsiasi.\
         - Puoi vedere gli oggetti che ti circondano e indicare approssimativamente dove sono sono."}
    ]

    messages_test = [
        {"role": "system", "content": "Sei un assistente virtuale collaborativo e parli italiano e inglese."}
    ]

    try:
        while True:

            gstt_resp = chat.send_gstt_req(start)
            print(gstt_resp.message)
            chat.get_logger().info('stt request complete')

            print(gstt_resp.message)

            #test_sing = "La frase \"Sai cantare una canzone?\" è simile alla frase \"" + gstt_resp.message + "\" ?. Rispondi solo con si o no."

            #test_sing = "Dimmi se la frase \"" + gstt_resp.message + "\" riguarda: \
             #   - una canzone\
              #  - l'abilità di saper cantare\
               # - la richiesta di cantare.\
                #Se una di queste condizioni è vera rispondi dicendo soltanto \"si\" altrimenti soltanto \"no\". "


            test_sing = "Dimmi se la frase \"" + gstt_resp.message + "\" riguarda la richiesta specifica di cantare\
                qualcosa ma non un'informazione sull'abilità generica di saper cantare.\
                Se è vero rispondi dicendo soltanto \"si\" altrimenti soltanto \"no\". "
            
            print(test_sing)

            messages_test.append({"role": "user", "content": test_sing})

            test_sing_reply = chat.get_response4(messages=messages_test)

            test_message = test_sing_reply['content']

            print (test_message.lower())

            if test_message.lower() == "sì." or test_message.lower() == "sì" or test_message.lower() == "si" or test_message.lower() == "si.":
                print("ok test riuscito")
            else:
                messages_nao.append({"role": "user", "content": gstt_resp.message})
                nao_reply = chat.get_response4(messages=messages_nao)

                gtts_resp = chat.send_gtts_req(nao_reply['content'])
                print(gtts_resp.debug)
                chat.get_logger().info(' tts Request complete')

                messages_nao.append(nao_reply)

            user_input = input("Premi dopo aver sentito la risposta")

    except KeyboardInterrupt:
        print("A presto!")

    chat.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()