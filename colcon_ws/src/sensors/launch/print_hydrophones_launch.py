from launch import LaunchDescription
from launch.actions import Node, LogInfo

def generate_launch_description():
    hydrophones_printer_node = Node(
        package='sensors',
        executable='print_hydrophones.py',
        name='hydrophones_printer',
        respawn=True,
        output='screen'
    )
    
    return LaunchDescription([
        hydrophones_printer_node
    ])