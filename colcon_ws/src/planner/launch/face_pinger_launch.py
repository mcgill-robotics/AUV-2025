from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the face_pinger node
    face_pinger_node = Node(
        package='planner',
        executable='face_pinger.py',
        name='face_pinger',
        respawn=False,
        output='screen'
    )

    # Return the LaunchDescription containing the node
    return LaunchDescription([
        face_pinger_node
    ])