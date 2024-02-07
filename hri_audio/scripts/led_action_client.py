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

import random

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from std_msgs.msg import ColorRGBA

from nao_lola_command_msgs.msg import *

from hri_interfaces.action import LedsPlay
from hri_interfaces.msg import LedIndexes
from hri_interfaces.msg import LedModes
from rclpy.duration import Duration


class LedsPlayActionClient(Node):

    def __init__(self):
        super().__init__( 'leds_play_action_client_'+str(random.randint(1, 100)) )
        self._action_client = ActionClient(self, LedsPlay, 'leds_play')

    def cancel_done(self, future):
        cancel_response = future.result()
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
        else:
            self.get_logger().info('Goal failed to cancel')

        rclpy.shutdown()    
        
    def goal_response_callback(self, future):
        self.get_logger().info('inside goal_response_callback')

        goal_handle = future.result()
        self.get_logger().info('goal_handle_ok')
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self._goal_handle = goal_handle

        self.get_logger().info('Goal accepted :)')

        # Start a 2 second timer
        self._timer = self.create_timer(5.0, self.timer_callback)

    def feedback_callback(self, feedback):
        self.get_logger().info('Received feedback:')
        #self.get_logger().info('Received feedback: {0}'.format(feedback.feedback.sequence))

    def timer_callback(self):
        self.get_logger().info('Canceling goal')
        # Cancel the goal
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)

        # Cancel the timer
        self._timer.cancel()

    def cancel_action(self):
        # Cancel the goal
        self.get_logger().info('Canceling goal')
        future = self._goal_handle.cancel_goal_async()
        future.add_done_callback(self.cancel_done)



    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = LedsPlay.Goal()
        goal_msg.leds = [LedIndexes.REYE, LedIndexes.LEYE]
        goal_msg.mode = LedModes.STEADY
        color = ColorRGBA()
        color.r=1.0 
        color.g=0.0 
        color.b=0.0

        for i in range(RightEyeLeds.NUM_LEDS):
            goal_msg.colors[i] = color;

        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def eyes_steady(self, color):

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = LedsPlay.Goal()
        goal_msg.leds = [LedIndexes.REYE, LedIndexes.LEYE]
        goal_msg.mode = LedModes.STEADY

        for i in range(RightEyeLeds.NUM_LEDS):
            goal_msg.colors[i] = color;

        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def ears_steady(self, intensities):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = LedsPlay.Goal()
        goal_msg.leds = [LedIndexes.REAR, LedIndexes.LEAR]
        goal_msg.mode = LedModes.STEADY
        #intensities = []
            #for i in range(RightEarLeds.NUM_LEDS):
        #    intensities.append(1.0)

        goal_msg.intensities=intensities

        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def chest_steady(self, color):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = LedsPlay.Goal()
        goal_msg.leds = [LedIndexes.CHEST]
        goal_msg.mode = LedModes.STEADY
        #color = ColorRGBA()
        #color.r=1.0 
        #color.g=1.0 
        #color.b=1.0

        goal_msg.colors[0] = color;

        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def head_steady(self, intensities):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = LedsPlay.Goal()
        goal_msg.leds = [LedIndexes.HEAD]
        goal_msg.mode = LedModes.STEADY
        #intensities = []
            #for i in range(RightEarLeds.NUM_LEDS):
        #    intensities.append(1.0)

        goal_msg.intensities=intensities

        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def head_blinking(self, frequency):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = LedsPlay.Goal()
        goal_msg.leds = [LedIndexes.HEAD]
        goal_msg.mode = LedModes.BLINKING
        intensities = []
        for i in range(HeadLeds.NUM_LEDS):
            intensities.append(1.0)

        goal_msg.intensities=intensities
        
        goal_msg.frequency=frequency

        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        self.get_logger().info('before send goal future server...')
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.get_logger().info('after send goal future server...')



def main(args=None):
    rclpy.init(args=args)

    #action_client1 = LedsPlayActionClient()
    action_client2 = LedsPlayActionClient()

    #intensities = []
    #for i in range(HeadLeds.NUM_LEDS):
    #    intensities.append(1.0)

    #action_client1.ears_steady(intensities)
    action_client2.head_blinking(1.0)
    #action_client2.get_clock().sleep_for(Duration(seconds=5.0))
    #action_client2.cancel_action()
    #action_client2.cancel_action()
    #action_client2.get_clock().sleep_for(Duration(seconds=5.0))
    #action_client2.cancel_action()
    #rclpy.spin(action_client1)
    rclpy.spin(action_client2)


    #action_client2.cancel_action()



if __name__ == '__main__':
   main()