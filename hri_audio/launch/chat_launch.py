from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nao_lola_client',
            #namespace='turtlesim2',
            executable='nao_lola_client',
            name='lola_node',
            output='screen'
        ),
        Node(
            package='sound_play',
            #namespace='turtlesim1',
            executable='soundplay_node.py',
            name='soundplay_node',
            output='screen'
        ),
        #Node(
        #    package='hri_audio',
        #    #namespace='turtlesim2',
        #    executable='gstt_service.py',
        #    name='gstt_service_node',
        #    output='screen'
        #),
        Node(
            package='hri_audio',
            #namespace='turtlesim2',
            executable='chat_service.py',
            name='chat_service_node',
            output='screen'
        ),
        Node(
            package='hri_audio',
            #namespace='turtlesim2',
            executable='gtts_service.py',
            name='gtts_service_node',
            output='screen'
        ),
        Node(
            package='hri_moves',
            #namespace='turtlesim1',
            executable='joints_play_action_server',
            name='joints_play_action_server',
            output='screen'
        ),
        Node(
            package='hri_moves',
            #namespace='turtlesim1',
            executable='led_action_server',
            name='led_action_server',
            output='screen'
        ),
        #Node(
        #    package='hri_moves',
        #    executable='chat_action_server',
        #    name='chat_action_server',
        #    output='screen',
        #),
        Node(
            package='hri_moves',
            executable='chat_action_client',
            name='chat_action_client',
            output='screen'
        ),
    ])