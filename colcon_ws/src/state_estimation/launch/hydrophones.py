from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the parameters for the node
    hydrophone_params = [
        {'hydrophones_dx': 0.1},
        {'hydrophones_dy': 0.1},
        {'hydrophones_dz': -0.1},
        {'hydrophones_time_unit': 0.0000001},
    ]

    # Create the launch description with the node
    return LaunchDescription([
        Node(
            package='state_estimation',
            executable='hydrophones_bearing.py',
            name='hydrophones_bearing',
            output='screen',
            respawn=True,
            parameters=hydrophone_params
        )
    ])

