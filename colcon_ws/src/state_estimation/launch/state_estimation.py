from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare arguments
    sim_arg = DeclareLaunchArgument('sim', default_value='false', description='Simulation mode')
    ekf_arg = DeclareLaunchArgument('ekf', default_value='true', description='Use EKF localization')

    # General parameters
    update_rate_param = {'update_rate': 100}
    z_pos_mount_offset_param = {'z_pos_mount_offset': 0}
    sensor_swap_warning_interval_param = {'sensor_swap_warning_interval': 5}

    # Include the hydrophones launch file
    hydrophones_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('state_estimation'), 'launch', 'hydrophones.launch.py')
        )
    )

    # Group for non-simulation mode
    non_sim_group = GroupAction(
        actions=[
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='dvl_transform_pub',
                arguments=[
                    LaunchConfiguration('auv_dvl_offset_x'),
                    LaunchConfiguration('auv_dvl_offset_y'),
                    LaunchConfiguration('auv_dvl_offset_z'),
                    LaunchConfiguration('q_dvlnominalup_dvlup_x'),
                    LaunchConfiguration('q_dvlnominalup_dvlup_y'),
                    LaunchConfiguration('q_dvlnominalup_dvlup_z'),
                    LaunchConfiguration('q_dvlnominalup_dvlup_w'),
                    'auv', 'dvl'
                ]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='imu_transform_pub',
                arguments=[
                    '0', '0', '0',
                    LaunchConfiguration('q_imunominalup_imuup_x'),
                    LaunchConfiguration('q_imunominalup_imuup_y'),
                    LaunchConfiguration('q_imunominalup_imuup_z'),
                    LaunchConfiguration('q_imunominalup_imuup_w'),
                    'auv', 'imu'
                ]
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='depth_transform_pub',
                arguments=['0', '0', '0', '0', '0', '0', '1', 'auv', 'depth']
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('state_estimation'), 'launch', 'localization.launch.py')
                ),
                condition=IfCondition(LaunchConfiguration('ekf')),
                launch_arguments={'remove_gravity': 'true'}.items()
            )
        ],
        condition=UnlessCondition(LaunchConfiguration('sim'))
    )

    # Group for simulation mode
    sim_group = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('state_estimation'), 'launch', 'localization.launch.py')
                ),
                condition=IfCondition(LaunchConfiguration('ekf')),
                launch_arguments={'remove_gravity': 'false'}.items()
            )
        ],
        condition=IfCondition(LaunchConfiguration('sim'))
    )

    # Return the launch description with all the actions
    return LaunchDescription([
        sim_arg,
        ekf_arg,
        hydrophones_launch,
        non_sim_group,
        sim_group,
        ParameterValue('update_state_on_clock', ParameterValue(LaunchConfiguration('sim'), value_type=bool)),
        ParameterValue('auv_dvl_offset_x', 0.0),
        ParameterValue('auv_dvl_offset_y', 0.0),
        ParameterValue('auv_dvl_offset_z', -0.3),
        ParameterValue('q_imunominalup_imuup_w', 0.0),
        ParameterValue('q_imunominalup_imuup_x', 0.0),
        ParameterValue('q_imunominalup_imuup_y', 0.0),
        ParameterValue('q_imunominalup_imuup_z', 1.0),
        ParameterValue('q_dvlnominalup_dvlup_w', 0.707),
        ParameterValue('q_dvlnominalup_dvlup_x', 0.0),
        ParameterValue('q_dvlnominalup_dvlup_y', 0.0),
        ParameterValue('q_dvlnominalup_dvlup_z', -0.707),
    ])
