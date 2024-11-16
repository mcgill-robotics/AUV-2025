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

    # Include bringup.launch.py
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

    # Include params.launch.py
    params_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('planner'),
                'launch',
                'params_launch.py'
            ])
        )
    )

    # Define the semis_dr_xy node
    semis_dr_xy_node = Node(
        package='planner',
        executable='semis_dr_xy.py',
        name='semis_dr_xy',
        respawn=False,
        output='screen'
    )

    # Return the LaunchDescription containing all components
    return LaunchDescription([
        bag_argument,
        bringup_launch,
        params_launch,
        semis_dr_xy_node
    ])
