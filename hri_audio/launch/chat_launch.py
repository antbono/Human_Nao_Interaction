from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hri_audio',
            #namespace='turtlesim2',
            executable='chat_node',
            name='chat_node'
        ),
        Node(
            package='sound_play',
            #namespace='turtlesim1',
            executable='soundplay_node',
            name='soundplay_node'
        ),
        Node(
            package='hri_audio',
            #namespace='turtlesim2',
            executable='gstt_service',
            name='gstt_service_node'
        ),
        Node(
            package='hri_audio',
            #namespace='turtlesim2',
            executable='gtts_service',
            name='gtts_service_node'
        )
    ])