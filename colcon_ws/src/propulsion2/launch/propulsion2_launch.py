from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import UnlessCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # sim argument needs to be added

    # microros launch needs ot be added

    thrust_mapper_node = Node(
        package='propulsion2',
        executable='thrust_mapper',
        name='thrust_mapper',
        parameters=[
            {'thruster_lower_limit': 1228},
            {'thruster_upper_limit': 1768},
            {'distance_thruster_thruster_length': 0.4},
            {'distance_thruster_thruster_width': 0.47},
            {'angle_thruster': 45},
            {'distance_thruster_middle_length': 0.0925}
        ],
        respawn=True,
        output='screen'
    )
    return LaunchDescription([
        thrust_mapper_node
    ])