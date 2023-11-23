# Import necessary Python modules
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('rosbag_record', default_value = 'true', choices = ['true', 'false'], description = "Enable recording all topics?"),
        # Launch the publisher node
        Node(
            package='beginner_tutorials',
            executable='talker',
            name='talker_node',
            output='screen',
            parameters=[{'time_period_int_ms': 5000}]
        ),
            ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('rosbag_record')),
            cmd=['ros2', 'bag', 'record', '-a'],
            shell=True
        )
    ])
