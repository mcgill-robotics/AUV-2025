from launch import LaunchDescription

from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import UnlessCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_argument = DeclareLaunchArgument(
        'sim',
        default_value='false'
    )
    thruster_serial_server_action = GroupAction(
        condition=UnlessCondition(LaunchConfiguration('sim')),
        actions=[   
            Node(
                package='rosserial_python',
                name='thrusters_serial_server',
                executable='serial_node.py',
                respawn=True,
                parameters=[
                    {'port': '/dev/power'},
                    {"baud": 115200}
                ]
            )
        ]   
    )
    thrust_mapper_node_parameters = {
        'thruster_PWM_lower_limit': 1228,
        'thruster_PWM_upper_limit': 1768,
        'distance_thruster_thruster_length': 0.4,
        'distance_thruster_thruster_width': 0.47,
        'angle_thruster': 45,
        'distance_thruster_middle_length': 0.0925
    }
    thrust_mapper_node = Node(
        package='propulsion',
        executable='thrust_mapper.py',
        name='thrust_mapper',
        parameters=[thrust_mapper_node_parameters],
        respawn=True,
        output='screen'
    )
    return LaunchDescription([
        use_sim_argument,
        thruster_serial_server_action,
        thrust_mapper_node
    ])