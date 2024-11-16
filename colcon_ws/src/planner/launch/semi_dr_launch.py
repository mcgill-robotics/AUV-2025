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
        description='Enable or disable bagging'
    )

    # Include the bringup.launch.py file
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

    # Include the params.launch.py file
    params_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('planner'),
                'launch',
                'params_launch.py'
            ])
        )
    )

    # Define the semis_dr node
    semis_dr_node = Node(
        package='planner',
        executable='semis_dr.py',
        name='semis_dr',
        respawn=False,
        output='screen'
    )

    # Return the LaunchDescription containing all elements
    return LaunchDescription([
        bag_argument,
        bringup_launch,
        params_launch,
        semis_dr_node
    ])
