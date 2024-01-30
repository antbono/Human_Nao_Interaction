from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nao_lola_client',
            #namespace='turtlesim2',
            executable='nao_lola_client',
            name='lola_node'
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
        ),
        Node(
            package='hri_moves',
            #namespace='turtlesim1',
            executable='joints_play_action_server',
            name='joints_play_action_server'
        ),
        Node(
            package='hri_audio',
            executable='chat_move_node',
            name='chat_move_node'
        ),
    ])