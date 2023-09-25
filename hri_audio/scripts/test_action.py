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

api_key = os.environ["OPENAI_API_KEY"]

openai.api_key = api_key

class ChatMoveNode(Node):

    def __init__(self):
        super().__init__('test_action_node')

        self.playing_move = False

        self._action_client = ActionClient(self, JointsPlay, 'joints_play')
        self.get_logger().info('test_action_node initializeeeed')

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
        self.get_logger().info('goal_response_callback inside')
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self.playing_move=True
        self.get_logger().info('playing move true)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
        return self._get_result_future

    def get_result_callback(self, future):
        result = future.result().result

        if(result.success):
            self.playing_move=False
            self.get_logger().info('playing move false')




def main(args=None):
    rclpy.init(args=args)

    chat = ChatMoveNode()
    start = True;
    messages = [
        {"role": "system", "content": "Sei un robot umanoide chiamato NAO e parli italiano. Ti piacciono i bambini. La tua casa è il laboratorio di robotica del DIMES all'Università della Calabria."}
    ]
    key_words = {"nao"}
    key_words_actions = {"nao": "hello.text"}
    sec_per_word=1;

    action_path="/home/toto/Gdrive/uni/robocup/robocup_ws/src/hri/hri_moves/moves/hello.txt"

    clock = chat.get_clock()
    t_start = clock.now()
    sec_start = t_start.seconds_nanoseconds()[0]
    try:
        while True:

         #   if not chat.playing_move:
         #      chat.send_goal(action_path)
         #      chat.get_clock().sleep_for(Duration(seconds=3))
         #   else:
         #       chat.get_clock().sleep_for(Duration(seconds=10))

            #user_input = input("Premi dopo aver sentito la risposta")

            t2 = clock.now()
            sec_2 = t2.seconds_nanoseconds()[0]

            print(t2.seconds_nanoseconds()[0])
            
            future_goal = chat.send_goal(action_path)
            rclpy.spin_until_future_complete(chat, future_goal)

            future_result = chat.goal_response_callback(future_goal)
            rclpy.spin_until_future_complete(chat, future_result)

            t3 = clock.now()
            sec_3 = t3.seconds_nanoseconds()[0]
            print(f"time elapsed: {sec_3-sec_2}")

    except KeyboardInterrupt:
        print("A presto!")

    chat.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()