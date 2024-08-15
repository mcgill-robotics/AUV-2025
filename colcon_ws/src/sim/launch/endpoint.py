from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('tcp_ip', default_value='0.0.0.0', description='TCP IP address'),
        DeclareLaunchArgument('tcp_port', default_value='10000', description='TCP port'),

        # Define the node
        Node(
            package='sim',
            executable='default_server_endpoint.py',
            name='unity_endpoint',
            output='screen',
            parameters=[{
                'tcp_ip': LaunchConfiguration('tcp_ip'),
                'tcp_port': LaunchConfiguration('tcp_port')
            }]
        )
    ])
