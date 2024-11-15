from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Node, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    port_argument = DeclareLaunchArgument('port', default_value='/dev/dvl', description='Serial port'),
    baudrate_argument = DeclareLaunchArgument('baudrate', default_value=115200, description='Serial baud rate'),

    waterlinked_driver_node = Node(
        package='sensors',
        executable='waterlinked_dvl.py',
        name='waterlinked_driver',
        output='screen',
        respawn=True,
        respawn_delay=1.0,
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'quat_variance': 0.0001
        }]
    )
    
    return LaunchDescription([
        port_argument,
        baudrate_argument,
        waterlinked_driver_node
    ])
