from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sim',
            executable='sim.x86_64',
            name='unity',
            arguments=['-batchmode', '-nographics'],
            output='log'
        )
    ])
