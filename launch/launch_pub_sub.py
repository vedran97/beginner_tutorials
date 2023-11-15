import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        # Launch the publisher node
        Node(
            package='beginner_tutorials',
            executable='talker',
            name='talker_node',
            output='screen',
        ),

        # Launch the subscriber node
        Node(
            package='beginner_tutorials',
            executable='listener',
            name='listener_node',
            output='screen',
        ),
    ])