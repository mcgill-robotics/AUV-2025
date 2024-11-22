from launch import LaunchDescription
from launch.actions import Node, LogInfo

def generate_launch_description():
    sensors_status_node = Node(
        package='sensors',
        executable='sensors_status.py',
        name='sensors_status',
        respawn=True,
        output='screen',
        parameters=[{
            'sensor_warning_interval': 5, # secs
            'sensor_status_update_rate': 10, # Hz
            'time_before_considered_inactive': 5, # secs
            'hydrophones_time_difference_tolerance': 5000000 # 0.5 secs
        }]  
    )
    
    return LaunchDescription([
        hydrophones_printer_node
    ])