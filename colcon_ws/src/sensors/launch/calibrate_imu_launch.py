from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import Node, LogInfo
from launch.substitutions import FindPackageShare

def generate_launch_description():
    sbg_device_mag_node = Node(
        package='sbg_driver',
        executable='sbg_device_mag',
        name='sbg_device_mag',
        output='screen',
        parameters=[PathJoinSubstitution(FindPackageShare('sensors'), 'config', 'sbg_imu.yaml')
    ])
    
    return LaunchDescription([
        sbg_device_mag_node
    ])