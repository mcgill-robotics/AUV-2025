from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetParameter
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Declare arguments
    sim_arg = DeclareLaunchArgument('sim', default_value='false', description='Simulation mode')
    vision_arg = DeclareLaunchArgument('vision', default_value='true', description='Vision argument')
    ekf_arg = DeclareLaunchArgument('ekf', default_value='true', description='EKF localization')

    # Group for EKF related parameters and nodes
    ekf_group = GroupAction(
        actions=[
            DeclareLaunchArgument('q_imunominalup_imuup_w', default_value='1.0'),
            DeclareLaunchArgument('q_imunominalup_imuup_x', default_value='0.0'),
            DeclareLaunchArgument('q_imunominalup_imuup_y', default_value='0.0'),
            DeclareLaunchArgument('q_imunominalup_imuup_z', default_value='0.0'),
            DeclareLaunchArgument('q_dvlnominalup_dvlup_w', default_value='1.0'),
            DeclareLaunchArgument('q_dvlnominalup_dvlup_x', default_value='0.0'),
            DeclareLaunchArgument('q_dvlnominalup_dvlup_y', default_value='0.0'),
            DeclareLaunchArgument('q_dvlnominalup_dvlup_z', default_value='0.0'),
            DeclareLaunchArgument('auv_dvl_offset_x', default_value='0.0'),
            DeclareLaunchArgument('auv_dvl_offset_y', default_value='0.0'),
            DeclareLaunchArgument('auv_dvl_offset_z', default_value='0.0'),
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
            )
        ],
        condition=IfCondition(LaunchConfiguration('ekf'))
    )

    # Include other launch files
    bringup_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('bringup'), 'launch', 'bringup.launch.py')
        ),
        launch_arguments={
            'sim': 'true',
            'vision': LaunchConfiguration('vision'),
            'ekf': LaunchConfiguration('ekf')
        }.items()
    )

    sim_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('sim'), 'launch', 'endpoint.launch.py')
        )
    )

    unity_bridge_node = Node(
        package='sim',
        executable='unity_bridge.py',
        name='unity_bridge',
        output='screen',
        parameters=[{'ekf': LaunchConfiguration('ekf')}]
    )

    return LaunchDescription([
        SetParameter(name='/use_sim_time', value='true'),
        sim_arg,
        vision_arg,
        ekf_arg,
        ekf_group,
        bringup_include,
        sim_include,
        unity_bridge_node
    ])
