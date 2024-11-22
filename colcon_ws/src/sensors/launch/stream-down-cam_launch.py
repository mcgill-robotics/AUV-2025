from launch import LaunchDescription
from launch.actions import Node, LogInfo

def generate_launch_description():
    down_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node',
        name='down_cam',
        output='screen',
        parameters=[{
            'video_device': '/dev/downcam',
            'image_width': 1280,
            'image_height': 720,
            'pixel_format': 'yuyv',
            'camera_frame_id': '/vision/down_cam',
            'io_method': 'mmap',
        }],
        remappings=[('/down_cam', '/vision/down_cam')]  # Remap /down_cam to /vision/down_cam
    )
    
    return LaunchDescription([
        down_cam_node
    ])