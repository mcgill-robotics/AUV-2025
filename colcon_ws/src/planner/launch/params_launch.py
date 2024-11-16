from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define the parameters as a single dictionary within a list
    node_parameters = [
        {
            'target_color': 'red',
            'nominal_depth': -2,
            'down_cam_search_depth': -2,
            'object_observation_time': 10,
            'linear_search_step_size': 2,
            'in_place_search_rotation_increment': 50,
            'bfs_expansion_size': 1,
            'mission_wait_time': 30,
            'buoy_centering_offset_distance': 2,
            'buoy_circumnavigation_radius': 1,
            'quali_gate_width': 2,
            'gate_width': 3,
            'red_side': 'left',
            'num_full_spins': 2,
            'octagon_closeness_threshold': 3,
            'object_search_time_limit': 180,
            'navigate_gate_time_limit': 120,
            'navigate_buoy_time_limit': 120,
            'navigate_lane_marker_time_limit': 120,
            'navigate_pinger_time_limit': 1800,
            'navigate_bin_time_limit': 120,
            'octagon_time_limit': 120,
            'trick_time_limit': 120,
            'center_dist_threshold': 20,
            'centering_delta_increment': 0.1
        }
    ]

    # Define the node with parameters
    mission_node = Node(
        package='planner',
        executable='missions.py',
        name='missions',
        respawn=False,
        output='screen',
        parameters=node_parameters
    )

    # Return the LaunchDescription containing the mission node
    return LaunchDescription([
        mission_node
    ])
