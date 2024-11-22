from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    launch_files = ['sensors_status_launch.py', 'tether_status_launch.py', 'depth_sensor_and_display_launch.py', 'dvl_launch.py', 'imu_launch.py', 'hydrophones_launch.py']
    include_files = [IncludeLaunchDescription(PathJoinSubstitution(FindPackageShare('sensors'), 'launch', launch_file)).items() for launch_file in launch_files]
    
    return LaunchDescription([
        DeclareLaunchArgument('vision', default_value='False'),
        include_IncludeLaunchDescription(PathJoinSubstitution(FindPackageShare('sensors'), 'launch', 'stream-down-cam_launch.py'),
                                        condition=IfCondition(LaunchConfiguration('vision'))).items(),
        include_IncludeLaunchDescription(PathJoinSubstitution(FindPackageShare('sensors'), 'launch', 'stream-front-cam_launch.py'),
                                        condition=IfCondition(LaunchConfiguration('vision'))).items()
    ].append(include_files))