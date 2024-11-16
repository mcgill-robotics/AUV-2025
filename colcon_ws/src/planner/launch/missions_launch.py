from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Include the params.launch file from the planner package
    params_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('planner'),
                'launch',
                'params.launch.py'
            ])
        )
    )

    # Define the missions node
    missions_node = Node(
        package='planner',
        executable='missions.py',
        name='missions',
        respawn=False,
        output='screen',
        parameters=[
            {
                'missions_to_objects': {
                    'find_gate': 'Gate',
                    'find_lane_marker': 'Lane Marker',
                    'find_buoy': 'Buoy',
                    'find_octagon': 'Octagon',
                    'navigate_dropper': 'Bins'
                },
                'missions_to_pinger_frequency': {
                    'find_gate': 0,
                    'find_lane_marker': 0,
                    'find_buoy': 40000,
                    'find_octagon': 30000,
                    'navigate_dropper': 0
                },
                'update_heading_time': 3,
                'advance_distance': 4
            }
        ]
    )

    # Return the LaunchDescription with all actions
    return LaunchDescription([
        params_launch,
        missions_node
    ])
