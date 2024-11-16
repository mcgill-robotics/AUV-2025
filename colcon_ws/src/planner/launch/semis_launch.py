from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare the 'bag' argument
    bag_argument = DeclareLaunchArgument(
        'bag',
        default_value='true',
        description='Enable bagging'
    )

    # Include the bringup.launch.py file from the bringup package
    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('bringup'),
                'launch',
                'bringup_launch.py'
            ])
        ),
        launch_arguments={'bag': LaunchConfiguration('bag')}.items()
    )

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

    # Define the semis node
    semis_node = Node(
        package='planner',
        executable='semis.py',
        name='semis',
        respawn=False,
        output='screen'
    )

    # Return the LaunchDescription with all components
    return LaunchDescription([
        bag_argument,
        bringup_launch,
        params_launch,
        semis_node
    ])
