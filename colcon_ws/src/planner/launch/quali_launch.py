from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Include bringup_launch.py
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ),
        launch_arguments={'bag': 'true'}.items()
    )

    # Include params_launch.py
    params_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('planner'),
                'launch',
                'params_launch.py'
            ])
        )
    )

    # Define the quali_node
    quali_node = Node(
        package='planner',
        executable='quali.py',
        name='quali',
        respawn=False,
        output='screen'
    )

    # Return the LaunchDescription
    return LaunchDescription([
        bringup_launch,
        params_launch,
        quali_node
    ])
