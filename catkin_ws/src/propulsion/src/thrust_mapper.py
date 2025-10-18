#!/usr/bin/env python3
"""
Description: Thrust mapper node subscribes to the effort topic, converts the wrench readings to thruster forces,
and then converts the forces to PWM signals and publishes them.
"""
import math
import numpy as np
import rospy
from thrust_mapper_utils import thruster_mount_dirs, force_to_pwm_thruster 
from auv_msgs.msg import ThrusterForces, ThrusterMicroseconds
from geometry_msgs.msg import Wrench, Vector3, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import WrenchStamped

# Constant parameters of the thruster positions
l = rospy.get_param("distance_thruster_thruster_length")
w = rospy.get_param("distance_thruster_thruster_width")
alpha = np.radians(rospy.get_param("angle_thruster"))
a = rospy.get_param("distance_thruster_middle_length")

# Matrix mapping from thruster forces to wrench (6x8) - need two matrices; one for sim one for rl.
T = np.array([
    # SURGE (X)
    [ np.cos(alpha), 0, 0, -np.cos(alpha), -np.cos(alpha), 0, 0,  np.cos(alpha)],
    # SWAY (Y)
    [ -np.sin(alpha), 0, 0, -np.sin(alpha), np.sin(alpha), 0, 0, np.sin(alpha)],
    # HEAVE (Z)
    [ 0, -1, -1, 0, 0, -1,-1,0],
    # ROLL (X-rotation)
    [0,  w/2,w/2,0,0, -w/2,-w/2,0],
    # PITCH (Y-rotation)
    [0,-a,a,0,0,a,-a,0],
    # YAW (Z-rotation)
    [ - (a*np.sin(alpha) - (w/2)*np.cos(alpha)),  0,  0, + (a*np.sin(alpha) - (w/2)*np.cos(alpha)),
      - (a*np.sin(alpha) - (w/2)*np.cos(alpha)),  0,  0, + (a*np.sin(alpha) - (w/2)*np.cos(alpha)) ]
])
T_inv = np.linalg.pinv(T)
#print("T =", T)
#print("T_inv =", T_inv)

# Temporary wait to allow sync with Arduino (adjust as needed)
rospy.sleep(4.0)


class ThrusterMapper:
    def __init__(self):
        
        # Publishers for thruster microseconds and forces
        self.pub_us = rospy.Publisher("/propulsion/microseconds", ThrusterMicroseconds, queue_size=1)
        self.pub_forces = rospy.Publisher("/propulsion/forces", ThrusterForces, queue_size=1)

        
        # Subscribe to the effort command topic
        rospy.Subscriber("/controls/effort", Wrench, self.wrench_to_thrust)

        #Buffer and Listener for reference frame transformation
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    
    
    def wrench_to_thrust(self, wrench_msg):
        """
        Callback function that maps a received Wrench message into thruster forces.
        It first converts the wrench from global to body frame, then applies the
        pseudo-inverse of the thruster mapping matrix. We assume that all messages published on /controls/effort
        are in the "auv" frame.

        """

        wrench_stamped = WrenchStamped()
        wrench_stamped.header.stamp = rospy.Time.now()
        wrench_stamped.header.frame_id = "auv"
        wrench_stamped.wrench = wrench_msg
        
        
        # Construct the 6x1 vector from the body wrench
        a_vec = np.array([
            [wrench_stamped.wrench.force.x],
            [wrench_stamped.wrench.force.y],
            [wrench_stamped.wrench.force.z],
            [wrench_stamped.wrench.torque.x],
            [wrench_stamped.wrench.torque.y],
            [wrench_stamped.wrench.torque.z]
        ])
        
        # Calculate the thruster forces using the pseudo-inverse
        converted_w = np.matmul(T_inv, a_vec)
        converted_w = (converted_w.flatten()).reshape((8,1))

        tf_msg = ThrusterForces()
        tf_msg.back_left = converted_w[0][0]
        tf_msg.heave_back_left = converted_w[1][0]
        tf_msg.heave_front_left = converted_w[2][0]
        tf_msg.front_left = converted_w[3][0]
        tf_msg.front_right = converted_w[4][0]
        tf_msg.heave_front_right = converted_w[5][0]
        tf_msg.heave_back_right = converted_w[6][0]
        tf_msg.back_right = converted_w[7][0]
        
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
        pwm_arr[ThrusterMicroseconds.BACK_LEFT] = force_to_pwm_thruster(1,forces_msg.back_left * thruster_mount_dirs[ThrusterMicroseconds.BACK_LEFT])
        pwm_arr[ThrusterMicroseconds.HEAVE_BACK_LEFT] = force_to_pwm_thruster(2,forces_msg.heave_back_left * thruster_mount_dirs[ThrusterMicroseconds.HEAVE_BACK_LEFT])
        pwm_arr[ThrusterMicroseconds.HEAVE_FRONT_LEFT] = force_to_pwm_thruster(3,forces_msg.heave_front_left * thruster_mount_dirs[ThrusterMicroseconds.HEAVE_FRONT_LEFT])
        pwm_arr[ThrusterMicroseconds.FRONT_LEFT] = force_to_pwm_thruster(4,forces_msg.front_left * thruster_mount_dirs[ThrusterMicroseconds.FRONT_LEFT])
        pwm_arr[ThrusterMicroseconds.FRONT_RIGHT] = force_to_pwm_thruster(5,forces_msg.front_right * thruster_mount_dirs[ThrusterMicroseconds.FRONT_RIGHT])
        pwm_arr[ThrusterMicroseconds.HEAVE_FRONT_RIGHT] = force_to_pwm_thruster(6,forces_msg.heave_front_right * thruster_mount_dirs[ThrusterMicroseconds.HEAVE_FRONT_RIGHT])
        pwm_arr[ThrusterMicroseconds.HEAVE_BACK_RIGHT] = force_to_pwm_thruster(7,forces_msg.heave_back_right * thruster_mount_dirs[ThrusterMicroseconds.HEAVE_BACK_RIGHT])
        pwm_arr[ThrusterMicroseconds.BACK_RIGHT] = force_to_pwm_thruster(8,forces_msg.back_right * thruster_mount_dirs[ThrusterMicroseconds.BACK_RIGHT])
    
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
