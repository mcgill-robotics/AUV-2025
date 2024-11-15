from launch import LaunchDescription
from launch_ros.actions import Node, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import ThisLaunchFileDir

def generate_launch_description():
    include_propulsion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [ThisLaunchFileDir(), '/propulsion_launch.py']
        ),
        launch_arguments = {
            # Empty
        }.items()
    )
    return LaunchDescription([
        include_propulsion_launch,
        Node(
            package='propulsion',
            executable='drytest',
            name='thrusters_test',
            output='screen',
            respawn=False
        )
    ])