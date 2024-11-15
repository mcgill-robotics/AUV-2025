from launch import LaunchDescription
from launch.actions import Node, LogInfo

def generate_launch_description():
    sbg_device_node = Node(
        package='sbg_driver',
        executable='sbg_device',
        name='sbg_device',
        respawn=True,
        output='screen',
        parameters=[{
            PathJoinSubstitution(FindPackageShare('sensors'), 'config', 'sbg_imu.yaml')
        }],
        remappings=[('/imu/data', '/sensors/imu/raw')]  # Remap /imu/data to /sensors/imu/raw
    )
    
    imu_republish_node = Node(
        package='sensors',
        executable='imu_republisher',
        name='imu_republish',
        output='screen',
        parameters=[{
            'angular_velocity_variance': 0.000000075,
            'acceleration_variance': 0.00005
        }]
    )
    
    return LaunchDescription([
        sbg_device_node,
        imu_republish_node
    ])
