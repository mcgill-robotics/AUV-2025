from launch import LaunchDescription
from launch.actions import Node, LogInfo

def generate_launch_description():
    display_serial_server_node = Node(
        package='rosserial_python', # This package needs to be switched to micro-ROS once it is setup on embedded end
        executable='serial_node.py',
        name='display_serial_server',
        respawn=True,
        respawn_delay=1.0,
        parameters=[{
            'port': '/dev/display',
            'baud': 115200
        }]
    )
    
    depth_republish_node = Node(
        package='sensors',
        executable='depth_republisher',
        name='depth_republish',
        output='screen'
        parameters=[{
            'variance': 0.0
        }]
    )
    
    return LaunchDescription([
        display_serial_server_node,
        depth_republish_node
    ])
