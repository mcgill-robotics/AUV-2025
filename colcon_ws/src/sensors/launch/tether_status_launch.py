from launch import LaunchDescription
from launch.actions import Node, LogInfo

def generate_launch_description():
    tether_status_node = Node(
        package='sensors',
        executable='tether_status.py',
        name='tether_status',
        output='screen',
        respawn=True,
        parameters=[{
            'ping_interval': 2, # 2 secs
            'ip_address_tether': '192.168.0.101' # this may need to be changed to 192.168.0.105
        }]
    )
    
    return LaunchDescription([
        tether_status_node
    ])