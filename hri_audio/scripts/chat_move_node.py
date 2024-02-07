#!/usr/bin/env python3
import re
import os
import openai
import sys
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from hri_interfaces.action import JointsPlay

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


from std_msgs.msg import *
from nao_lola_command_msgs.msg import * 

from led_action_client import LedsPlayActionClient

api_key = os.environ["OPENAI_API_KEY"]

openai.api_key = api_key

class ChatMoveNode2(Node):

    white = ColorRGBA()
    white.r = 1.0
    white.g = 1.0
    white.b = 1.0

    def __init__(self):
        super().__init__('chat_move_node')

        self.playing_move = False

        self.gstt_client = self.create_client(SetBool, 'gstt_service')
        while not self.gstt_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('gstt_service not available, waiting again...')
        self.gstt_req = SetBool.Request()

        self.gtts_client = self.create_client(TextToSpeech, 'gtts_service')
        while not self.gtts_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('gtts_service not available, waiting again...')
        self.gtts_req = TextToSpeech.Request()

        self._action_client = ActionClient(self, JointsPlay, 'joints_play')

        self.chest_pub = self.create_publisher(ChestLed, 'effectors/chest_led', 10)
        self.right_eye_pub = self.create_publisher(RightEyeLeds, 'effectors/right_eye_leds', 10)
        self.left_eye_pub = self.create_publisher(LeftEyeLeds, 'effectors/left_eye_leds', 10)
        self.right_ear_pub = self.create_publisher(RightEarLeds, 'effectors/right_ear_leds', 10)
        self.left_ear_pub = self.create_publisher(LeftEarLeds, 'effectors/left_ear_leds', 10)
        self.head_pub = self.create_publisher(HeadLeds, 'effectors/head_leds', 10)

        self.get_logger().info('chat_move_node initialized')


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

    def send_goal(self, path):
        goal_msg = JointsPlay.Goal()
        goal_msg.path = path

        self._action_client.wait_for_server()

        self.get_logger().info('action server available')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self.get_logger().info('goal sent')
        return self._send_goal_future
        #self._send_goal_future.add_done_callback(self.goal_response_callback)
        #self.get_logger().info('response callback added')

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self.playing_move=True
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        return self._get_result_future

    def get_result_callback(self, future):
        result = future.result().result

        if(result.success):
            self.playing_move=False

    def chest_on(self, color=white):
        chest_msg = ChestLed()
        chest_msg.color = color        
        self.chest_pub.publish(chest_msg)

    def right_eye_on(self, color=white):
        right_eye_msg = RightEyeLeds()
        for i in range(right_eye_msg.NUM_LEDS): #8 leds 
            right_eye_msg.colors[i]=color
        self.right_eye_pub.publish(right_eye_msg)

    def left_eye_on(self, color=white):
        left_eye_msg = LeftEyeLeds()
        for i in range(left_eye_msg.NUM_LEDS):
            left_eye_msg.colors[i]=color
        self.left_eye_pub.publish(left_eye_msg)

    def eyes_on(self, color=white):
        self.right_eye_on(color)
        self.left_eye_on(color)

    #def right_ear(self, intensity=1.0, loop=false):
    #    right_ear_msg = RightEarLeds()

    #    if loop and intensity>0.0:

    #    else:
    #        for i in range(right_ear_msg.NUM_LEDS): #10 leds
    #            right_ear_msg.intensities[i]=intensity
    #        self.right_ear_pub.publish(right_ear_msg)


        

    #def left_ear_on(self):
        #left_ear_msg = LeftEarLeds()
        #for i in range(left_ear_msg.NUM_LEDS):
        #    left_ear_msg.intensities[i]=1.0
        #self.left_ear_pub.publish(left_ear_msg)

    #def ears_on(self):
    #    self.right_ear_on()
    #    self.left_ear_on()

    #def head_on(self):
    #    head_msg = HeadLeds()
    #    for i in range(head_msg.NUM_LEDS): # 12 leds
    #        head_msg.intensities[i]=1.0
     #   self.head_pub.publish(head_msg)


def main(args=None):
    rclpy.init(args=args)

    chat = ChatMoveNode2()
    start = True;
    messages = [
        {"role": "system", "content": """Sei un robot umanoide chiamato NAO e parli italiano. Ti piacciono i bambini. 
                                        La tua casa è il laboratorio di robotica del DIMES all'Università della Calabria.
                                        Per motivi di sicurezza oggi non ti muovi nell'ambiente."""}
    ]
    key_words = {"ciao","tu","te","grande","piccolo","sotto","sopra","destra","sinistra","paura"}
    key_words_actions = {"ciao": "/home/nao/rolling_ws/src/hri/hri_moves/moves/hello.txt",
                         "tu": "/home/nao/rolling_ws/src/hri/hri_moves/moves/you.txt",
                         "te": "/home/nao/rolling_ws/src/hri/hri_moves/moves/you.txt",
                         "grande": "/home/nao/rolling_ws/src/hri/hri_moves/moves/big.txt",
                         "piccolo": "/home/nao/rolling_ws/src/hri/hri_moves/moves/little.txt",
                         "sotto": "/home/nao/rolling_ws/src/hri/hri_moves/moves/down.txt",
                         "sopra": "/home/nao/rolling_ws/src/hri/hri_moves/moves/up.txt",
                         "destra": "/home/nao/rolling_ws/src/hri/hri_moves/moves/right.txt",
                         "sinistra": "/home/nao/rolling_ws/src/hri/hri_moves/moves/left.txt",
                         "paura": "/home/nao/rolling_ws/src/hri/hri_moves/moves/fear.txt",
                         }
    sec_per_word=0.5;
    delta=5

    #ledClient = LedsPlayActionClient()

    w = ColorRGBA()
    w.r = 1.0
    w.g = 1.0
    w.b = 1.0

    chat.eyes_on(w)

    #intensities_on = []
    #for i in range(HeadLeds.NUM_LEDS):
    #    intensities_on.append(1.0)

    #intensities_off = []
    #for i in range(HeadLeds.NUM_LEDS):
    #    intensities_off.append(0.0)

    #ledClient.eyes_steady(w)
    #ledClient.chest_steady(w)
    #ledClient.ears_steady(intensities_on)


    try:
        while True:
                
                blinking_client = LedsPlayActionClient()
                num_words = 0
                key_words_found = []
                key_words_time = []
                playing = False
                lastActionTime = 0

                chat.get_logger().info('ready to listen')
                gstt_resp = chat.send_gstt_req(start)
                #print(f"risultato service: {gstt_resp.success}")
                chat.get_logger().info('stt request complete')
                blinking_client.head_blinking(1.0)
               
                messages.append({"role": "user", "content": gstt_resp.message})
                new_message = chat.get_response(messages=messages)
                

                reply_text = new_message['content'];
                print("nao reply: ")
                print(reply_text)
                blinking_client.cancel_action()
                # using regex( findall() )
                # to extract words from string
                words = re.findall(r'\w+', reply_text)
                
                
                firstWord=True
                for w in words:
                    num_words += 1
                    w = w.lower()
                    if w in key_words:
                        key_words_found.append(w)
                        if (num_words<=delta) and firstWord :
                            #key_words_time.append(num_words*sec_per_word)
                            key_words_time.append(0.1)
                            firstWord = False
                        elif (num_words<=delta) and not firstWord:
                                key_words_time.append(num_words*sec_per_word)
                        else:
                            key_words_time.append((num_words-delta)*sec_per_word)  

                
                #speaking
                gtts_resp = chat.send_gtts_req(reply_text)
                #print(gtts_resp.debug)
                chat.get_logger().info('tts Request complete')

                clock = chat.get_clock()
                t_start = clock.now().seconds_nanoseconds()[0] #seconds

                for i in range(len(key_words_time)):
                    print(i)
                    t_word = key_words_time[i] + t_start # seconds
                    t_cur = clock.now().seconds_nanoseconds()[0]

                    if(t_cur < t_word):
                        # wait
                        sleep_for = t_word - t_cur
                        chat.get_logger().info(f"waiting next action for: {sleep_for}")
                        ta = clock.now().seconds_nanoseconds()[0]
                        chat.get_clock().sleep_for(Duration(seconds=sleep_for))
                        tb = clock.now().seconds_nanoseconds()[0]
                        chat.get_logger().info(f"slept for: {tb-ta}")
                        # execute
                        action_path = key_words_actions[key_words_found[i]]
                        t1 = clock.now().seconds_nanoseconds()[0]

                        future_goal = chat.send_goal(action_path)
                        rclpy.spin_until_future_complete(chat, future_goal)
                       # chat.get_logger().info("future_goal ok")
                        future_result = chat.goal_response_callback(future_goal)
                        rclpy.spin_until_future_complete(chat, future_result)
                        #chat.get_logger().info("future_result ok")
                        t2 = clock.now().seconds_nanoseconds()[0]
                        chat.get_logger().info(f"Last action duration: {t2-t1}")

                    
                messages.append(new_message)
                #rclpy.spin_until_future_complete(chat, gtts_resp)

                user_input = input("Premi dopo aver sentito la risposta")

    except KeyboardInterrupt:
        print("A presto!")

    chat.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()