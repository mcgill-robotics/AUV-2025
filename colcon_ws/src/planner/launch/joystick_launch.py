from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the joystick node
    joystick_node = Node(
        package='planner',
        executable='joystick.py',
        name='joystick',
        respawn=False,
        output='screen'
    )

    # Return the LaunchDescription containing the node
    return LaunchDescription([
        joystick_node
    ])
