from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import UnlessCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # sim argument needs to be added

    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=["serial", "--dev", "/dev/ttyACM0", "--baud-rate", "115200"],
        output='screen'
    )

    thrust_mapper_node = Node(
        package='propulsion',
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
        micro_ros_agent_node,
        thrust_mapper_node
    ])
