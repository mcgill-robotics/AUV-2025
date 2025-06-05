#!/usr/bin/env python3
import rospy
from substates.utility.controller import Controller
import numpy as np
from scipy.interpolate import make_splprep


def generate_trajectory(d=5.0, r=2.0, num_circle_pts=50, transition_pts=50):
    waypoints = []

    # 1. Submerge from surface to 0.5m depth (along Z)
    z_surface = np.linspace(0.0, -0.5, transition_pts)
    for z in z_surface:
        waypoints.append([0, 0, z])  # X=0, Y=0

    # 2. Move forward in +X (North) at constant depth
    x_forward = np.linspace(0, d, transition_pts)
    for x in x_forward:
        waypoints.append([x, 0, -0.5])  # Y=0

    # 3. Semi-circle turn (XY plane, counter-clockwise)
    theta = np.linspace(0, np.pi, num_circle_pts)
    circle_x = d + r * np.sin(theta)        # From d to d (center) + r*sin()
    circle_y = r - r * np.cos(theta)        # From 0 to 2r
    circle_z = -0.5 * np.ones_like(theta)   # Constant depth

    for x, y, z in zip(circle_x, circle_y, circle_z):
        waypoints.append([x, y, z])

    # 4. Return back along -X (North) at Y=2r
    x_back = np.linspace(d, 0, transition_pts)
    for x in x_back:
        waypoints.append([x, 2 * r, -0.5])

    # Convert to NumPy array
    waypoints = np.array(waypoints)
    x, y, z = waypoints[:, 0], waypoints[:, 1], waypoints[:, 2]

    # Remove near-duplicate points to avoid spline fitting error
    unique_waypoints = [waypoints[0]]
    for pt in waypoints[1:]:
        if np.linalg.norm(pt - unique_waypoints[-1]) > 1e-6:
            unique_waypoints.append(pt)
    waypoints = np.array(unique_waypoints)
    x, y, z = waypoints[:, 0], waypoints[:, 1], waypoints[:, 2]

    # Fit spline with light smoothing
    spline, u = make_splprep([x, y, z], s=1e-6)
    return spline, waypoints



def main():

    rospy.init_node("Prequal_controller")
    controls = Controller(rospy.Time(0))    

    try:
        Path, waypoints = generate_trajectory(d = 2.0, r = 0.75)
        controls.LoS(Path, 0.5)

    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt received. Stopping movements.")
    finally:
        rospy.loginfo("Shutting down controls…")
        controls.kill()



if __name__ == "__main__":
    main()
