from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Define the output directory using os.path.join
    output_dir = os.path.join(
        os.getenv('HOME'), 'ros2_ws/src/state_estimation/data'
    )

    # Create the launch description
    return LaunchDescription([
        Node(
            package='state_estimation',
            executable='data_format.py',
            name='data_collection',
            output='screen',
            parameters=[
                {'update_rate': 100},
                {'radius_earth': 6378.1370},
                {'laditude_offset': 0},
                {'longitude_offset': 0},
                {'frame_rate': 10},
                {'output_dir': output_dir}
            ],
        )
    ])
