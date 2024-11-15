from launch import LaunchDescription
from launch.actions import Node, LogInfo

def generate_launch_description():
    hydrophones_serial_server_node = Node(
        package='rosserial_python', # This package needs to be switched to micro-ROS once it is setup on embedded end
        executable='serial_node.py',
        name='hydrophones_serial_server',
        respawn=True,
        respawn_delay=1.0,
        parameters=[{
            'port': '/dev/display'
            'baud': 115200
        }]
    )
    
    return LaunchDescription([
        hydrophones_serial_server_node
    ])
