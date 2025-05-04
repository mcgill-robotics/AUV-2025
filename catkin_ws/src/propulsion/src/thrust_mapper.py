#!/usr/bin/env python3
"""
Description: Thrust mapper node subscribes to the effort topic, converts the wrench readings to forces,
and then converts the forces to PWM signals and publishes them.
"""

import numpy as np
import rospy
from thrust_mapper_utils import (
    thruster_mount_dirs,
    force_to_pwm_thruster1,
    force_to_pwm_thruster2,
    force_to_pwm_thruster3,
    force_to_pwm_thruster4,
    force_to_pwm_thruster5,
    force_to_pwm_thruster6,
    force_to_pwm_thruster7,
    force_to_pwm_thruster8,
)  # Assumes force_to_pwm, thruster_mount_dirs, etc. are defined here.
from auv_msgs.msg import ThrusterForces, ThrusterMicroseconds
from geometry_msgs.msg import Wrench, Vector3
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# Constant parameters of the thruster positions
l = rospy.get_param("distance_thruster_thruster_length")
w = rospy.get_param("distance_thruster_thruster_width")
alpha = np.radians(rospy.get_param("angle_thruster"))
a = rospy.get_param("distance_thruster_middle_length")

# Matrix mapping from thruster forces to wrench (6x8)
T = np.array([
    # SURGE (X)
    [ np.cos(alpha),  0,0, -np.cos(alpha), -np.cos(alpha),0,0,  np.cos(alpha)],
    # SWAY (Y)
    [ np.sin(alpha),0,0,  np.sin(alpha), -np.sin(alpha),0,0, -np.sin(alpha)],
    # HEAVE (Z)
    [0,1,1,0,0,1,1,0],
    # ROLL (X-rotation)
    [0,  w/2,w/2,0,0, -w/2,-w/2,0],
    # PITCH (Y-rotation)
    [0,a,-a,0,0,   -a,a,0],
    # YAW (Z-rotation)
    [ - (a*np.sin(alpha) + (w/2)*np.cos(alpha)),  0,  0,  (a*np.sin(alpha) + (w/2)*np.cos(alpha)),
      - (a*np.sin(alpha) + (w/2)*np.cos(alpha)),  0,  0,  (a*np.sin(alpha) + (w/2)*np.cos(alpha)) ]
])
T_inv = np.linalg.pinv(T)
print("T =", T)
print("T_inv =", T_inv)

# Temporary wait to allow sync with Arduino (adjust as needed)
rospy.sleep(4.0)


class ThrusterMapper:
    def __init__(self):
        # Initialize current orientation [roll, pitch, yaw]
        self.current_orientation = np.zeros(3)
        rospy.Subscriber("/state", Odometry, self.orientation_cb)
        
        # Publishers for thruster microseconds and forces
        self.pub_us = rospy.Publisher("/propulsion/microseconds", ThrusterMicroseconds, queue_size=1)
        self.pub_forces = rospy.Publisher("/propulsion/forces", ThrusterForces, queue_size=1)
        
        # Subscribe to the effort command topic
        rospy.Subscriber("/controls/effort", Wrench, self.wrench_to_thrust)
    
    def orientation_cb(self, msg):
        """Callback to update the current orientation from Odometry."""
        q = msg.pose.pose.orientation
        self.current_orientation = euler_from_quaternion([q.x, q.y, q.z, q.w])
    
    def global_to_body_frame(self, wrench):
        yaw = self.current_orientation[2]
        R = np.array([
            [np.cos(yaw), np.sin(yaw), 0],   # Global X → Body X
            [-np.sin(yaw), np.cos(yaw), 0],  # Global Y → Body Y
            [0, 0, 1]                        # Global Z → Body Z
        ])
        # For relative commands, use identity matrix
        force_body = np.array([wrench.force.x, wrench.force.y, wrench.force.z])
        torque_body = np.array([wrench.torque.x, wrench.torque.y, wrench.torque.z])
        return Wrench(force=Vector3(*force_body), torque=Vector3(*torque_body))


    def wrench_to_thrust(self, wrench_msg):
        """
        Callback function that maps a received Wrench message into thruster forces.
        It first converts the wrench from global to body frame, then applies the
        pseudo-inverse of the thruster mapping matrix.
        """
        body_wrench = self.global_to_body_frame(wrench_msg)
        
        # Construct the 6x1 vector from the body wrench
        a_vec = np.array([
            [body_wrench.force.x],
            [body_wrench.force.y],
            [body_wrench.force.z],
            [body_wrench.torque.x],
            [body_wrench.torque.y],
            [body_wrench.torque.z]
        ])
        
        # Calculate the thruster forces using the pseudo-inverse
        converted_w = np.matmul(T_inv, a_vec)
        tf_msg = ThrusterForces()
        tf_msg.BACK_LEFT = converted_w[0][0]
        tf_msg.HEAVE_BACK_LEFT = converted_w[1][0]
        tf_msg.HEAVE_FRONT_LEFT = converted_w[2][0]
        tf_msg.FRONT_LEFT = converted_w[3][0]
        tf_msg.FRONT_RIGHT = converted_w[4][0]
        tf_msg.HEAVE_FRONT_RIGHT = converted_w[5][0]
        tf_msg.HEAVE_BACK_RIGHT = converted_w[6][0]
        tf_msg.BACK_RIGHT = converted_w[7][0]
        
        # Publish the computed thruster forces (useful for simulation/debugging)
        self.pub_forces.publish(tf_msg)
        
        # Convert forces to PWM signals and publish them
        self.forces_to_pwm_publisher(tf_msg)
    
    def forces_to_pwm_publisher(self, forces_msg):
        """
        Converts thruster forces into PWM signals and publishes them.
        Applies individual limits to prevent overcurrent.
        """
        pwm_arr = [None] * 8
        pwm_arr[ThrusterMicroseconds.BACK_LEFT] = force_to_pwm_thruster1(forces_msg.BACK_LEFT * thruster_mount_dirs[ThrusterMicroseconds.BACK_LEFT])
        pwm_arr[ThrusterMicroseconds.HEAVE_BACK_LEFT] = force_to_pwm_thruster2(forces_msg.HEAVE_BACK_LEFT * thruster_mount_dirs[ThrusterMicroseconds.HEAVE_BACK_LEFT])
        pwm_arr[ThrusterMicroseconds.HEAVE_FRONT_LEFT] = force_to_pwm_thruster3(forces_msg.HEAVE_FRONT_LEFT * thruster_mount_dirs[ThrusterMicroseconds.HEAVE_FRONT_LEFT])
        pwm_arr[ThrusterMicroseconds.FRONT_LEFT] = force_to_pwm_thruster4(forces_msg.FRONT_LEFT * thruster_mount_dirs[ThrusterMicroseconds.FRONT_LEFT])
        pwm_arr[ThrusterMicroseconds.FRONT_RIGHT] = force_to_pwm_thruster5(forces_msg.FRONT_RIGHT * thruster_mount_dirs[ThrusterMicroseconds.FRONT_RIGHT])
        pwm_arr[ThrusterMicroseconds.HEAVE_FRONT_RIGHT] = force_to_pwm_thruster6(forces_msg.HEAVE_FRONT_RIGHT * thruster_mount_dirs[ThrusterMicroseconds.HEAVE_FRONT_RIGHT])
        pwm_arr[ThrusterMicroseconds.HEAVE_BACK_RIGHT] = force_to_pwm_thruster7(forces_msg.HEAVE_BACK_RIGHT * thruster_mount_dirs[ThrusterMicroseconds.HEAVE_BACK_RIGHT])
        pwm_arr[ThrusterMicroseconds.BACK_RIGHT] = force_to_pwm_thruster8(forces_msg.BACK_RIGHT * thruster_mount_dirs[ThrusterMicroseconds.BACK_RIGHT])
    
        # Retrieve PWM limits from parameters
        thruster_lower_limit = rospy.get_param("thruster_PWM_lower_limit")
        thruster_upper_limit = rospy.get_param("thruster_PWM_upper_limit")
    
        # Apply limit checking for each thruster
        for i in range(len(pwm_arr)):
            if pwm_arr[i] > thruster_upper_limit:
                pwm_arr[i] = thruster_upper_limit
                rospy.logwarn("INDIVIDUAL FUSE EXCEEDED: Thruster %d", i + 1)
            elif pwm_arr[i] < thruster_lower_limit:
                pwm_arr[i] = thruster_lower_limit
                rospy.logwarn("INDIVIDUAL FUSE EXCEEDED: Thruster %d", i + 1)
    
        pwm_msg = ThrusterMicroseconds(pwm_arr)
        self.pub_us.publish(pwm_msg)
    
    def re_arm(self):
        """
        Sends the arming signal to the thrusters upon startup.
        """
        rospy.sleep(1)
        msg1 = ThrusterMicroseconds([1500] * 8)
        msg2 = ThrusterMicroseconds([1540] * 8)
    
        self.pub_us.publish(msg1)
        rospy.sleep(0.5)
        self.pub_us.publish(msg2)
        rospy.sleep(0.5)
        self.pub_us.publish(msg1)
    
    def shutdown(self):
        """
        Turns off the thrusters when the node is shutting down.
        """
        msg = ThrusterMicroseconds([1500] * 8)
        self.pub_us.publish(msg)


if __name__ == "__main__":
    rospy.init_node("thrust_mapper")
    mapper = ThrusterMapper()
    rospy.on_shutdown(mapper.shutdown)
    mapper.re_arm()
    rospy.spin()
