from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Include the params.launch.py file from the planner package
    params_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('planner'),
                'launch',
                'params_launch.py'
            ])
        )
    )

    # Define the quali_vision node
    quali_vision_node = Node(
        package='planner',
        executable='quali_vision.py',
        name='quali_vision',
        respawn=False,
        output='screen'
    )

    # Return the LaunchDescription containing the components
    return LaunchDescription([
        params_launch,
        quali_vision_node
    ])
