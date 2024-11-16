from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Node, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    port_argument = DeclareLaunchArgument('port', default_value='/dev/dvl', description='Serial port'),
    baudrate_argument = DeclareLaunchArgument('baudrate', default_value=115200, description='Serial baud rate'),

    calibrate_dvl_node = Node(
        package='sensors',
        executable='calibrate_dvl.py',
        name='calibrate_dvl',
        output='screen',
        respawn=True,
        respawn_delay=5.0,
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate')
        }]
    )
    
    return LaunchDescription([
        port_argument,
        baudrate_argument,
        calibrate_dvl_node
    ])
