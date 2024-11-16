from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the pooltest node
    pooltest_node = Node(
        package='planner',
        executable='pooltest.py',
        name='pooltest',
        respawn=False,
        output='screen'
    )

    # Return the LaunchDescription containing the node
    return LaunchDescription([
        pooltest_node
    ])