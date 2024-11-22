from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare



def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('vision', default_value='False')

        include_IncludeLaunchDescription(PathJoinSubstitution(FindPackageShare('sensors'), 'launch', 'sensors_status_launch.py')).items()
        include_IncludeLaunchDescription(PathJoinSubstitution(FindPackageShare('sensors'), 'launch', 'tether_status_launch.py')).items()
        include_IncludeLaunchDescription(PathJoinSubstitution(FindPackageShare('sensors'), 'launch', 'depth_sensor_and_display_launch.py')).items()
        include_IncludeLaunchDescription(PathJoinSubstitution(FindPackageShare('sensors'), 'launch', 'dvl_launch.py')).items()
        include_IncludeLaunchDescription(PathJoinSubstitution(FindPackageShare('sensors'), 'launch', 'imu_launch.py')).items()
        include_IncludeLaunchDescription(PathJoinSubstitution(FindPackageShare('sensors'), 'launch', 'hydrophones_launch.py')).items()

        include_IncludeLaunchDescription(PathJoinSubstitution(FindPackageShare('sensors'), 'launch', 'stream-down-cam_launch.py')
                                        condition=IfCondition(LaunchConfiguration('vision'))).items()
        include_IncludeLaunchDescription(PathJoinSubstitution(FindPackageShare('sensors'), 'launch', 'stream-front-cam_launch.py')
                                        condition=IfCondition(LaunchConfiguration('vision'))).items()
    ])