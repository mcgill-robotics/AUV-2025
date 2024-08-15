from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the remove_gravity argument with a default value
    remove_gravity_arg = DeclareLaunchArgument(
        'remove_gravity',
        default_value='true',
        description='Remove gravitational acceleration from IMU data'
    )

    # Define the ekf_localization node with parameters and rosparams
    ekf_localization_node = Node(
        package='robot_localization',
        executable='ekf_localization_node',
        name='ekf_localization',
        output='screen',
        parameters=[
            {'frequency': 10},
            {'sensor_timeout': 1.0},
            {'two_d_mode': False},
            {'print_diagnostics': True},
            {'map_frame': 'map'},
            {'odom_frame': 'odom'},
            {'base_link_frame': 'auv'},
            {'world_frame': 'odom'},
            {'twist0': '/sensors/dvl/twist'},
            {'imu0': '/sensors/imu/data'},
            {'depth0': '/sensors/depth/pose'},
            {'debug': True},
            {'depth0_relative': False},
            {'imu0_remove_gravitational_acceleration': 
                LaunchConfiguration('remove_gravity')}
        ],
        # Load rosparams
        remappings=[],
        ros_parameters=[
            {'twist0_config': [
                False, False, False, 
                False, False, False, 
                True, True, True, 
                False, False, False,
                False, False, False
            ]},
            {'imu0_config': [
                False, False, False, 
                True,  True,  True, 
                False, False, False, 
                True,  True,  True,
                True, True, True
            ]},
            {'depth0_config': [
                False, False, True, 
                False, False, False, 
                False, False, False, 
                False, False, False,
                False, False, False
            ]}
        ]
    )

    # Define the odom_republisher node
    odom_republisher_node = Node(
        package='state_estimation',
        executable='odom_republisher',
        name='odom_republisher',
        output='screen'
    )

    # Create the launch description and add the nodes
    return LaunchDescription([
        remove_gravity_arg,
        ekf_localization_node,
        odom_republisher_node
    ])
