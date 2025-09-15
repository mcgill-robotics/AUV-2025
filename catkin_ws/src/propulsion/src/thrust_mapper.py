#!/usr/bin/env python3
"""
Description: Thrust mapper node subscribes to the effort topic, converts the wrench readings to thruster forces,
and then converts the forces to PWM signals and publishes them.
"""
import numpy as np
import rospy
from thrust_mapper_utils import force_to_pwm_thruster 
from auv_msgs.msg import ThrusterForces, ThrusterMicroseconds
from geometry_msgs.msg import Wrench

# Constant parameters of the thruster positions
#TODO: Replace these parameters with a,b,c,d,e 
l = rospy.get_param("distance_thruster_thruster_length")
w = rospy.get_param("distance_thruster_thruster_width")
alpha = np.radians(rospy.get_param("angle_thruster"))
a = rospy.get_param("distance_thruster_middle_length")

# Matrix mapping from thruster forces to wrench (6x8) 
T = np.array([
    # SURGE (X)
    [ cos45, 0, 0, -cos45, -cos45, 0, 0, cos45],
    # SWAY (Y)
    [ -sin45, 0, 0, -sin45, sin45, 0, 0, sin45],
    # HEAVE (Z)
    [ 0, -1, -1, 0, 0, -1, -1, 0],
    # ROLL (X-rotation)
    [ sin45*e, b, b, sin45*e, -sin45*e, -b, -b, -sin45*e],
    # PITCH (Y-rotation)
    [ cos45*e, -a, a, -cos45*e, -cos45*e, a, -a, cos45*e],
    # YAW (Z-rotation)
    [ (cos45*c+sin45*d), 0, 0, -(cos45*c+sin45*d), (cos45*c+sin45*d), 0, 0, -(cos45*c+sin45*d)]
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


        # Retrieve PWM limits from parameters
        self.thruster_lower_limit = rospy.get_param("thruster_PWM_lower_limit")
        self.thruster_upper_limit = rospy.get_param("thruster_PWM_upper_limit")
        
        # Subscribe to the effort command topic
        rospy.Subscriber("/controls/effort", Wrench, self.wrench_to_thrust, queue_size = 1)


    def wrench_to_thrust(self, wrench_msg):
        """
        Callback function that maps a received Wrench message into thruster forces by applying the
        pseudo-inverse of the thruster mapping matrix. We assume that all messages published on /controls/effort
        are in the "auv" frame.

        """
            
        # Construct the (6,) row vector from the body wrench
        wrench_vec = np.array([
            wrench_msg.force.x,
            wrench_msg.force.y,
            wrench_msg.force.z,
            wrench_msg.torque.x,
            wrench_msg.torque.y,
            wrench_msg.torque.z
        ])
        
        # Calculate the thruster forces using the pseudo-inverse
        thrust_forces = np.matmul(T_inv, wrench_vec) # Shape (8,)

        tf_msg = ThrusterForces()
        tf_msg.BACK_RIGHT = thrust_forces[0]
        tf_msg.HEAVE_BACK_RIGHT = thrust_forces[1]
        tf_msg.HEAVE_FRONT_RIGHT = thrust_forces[2]
        tf_msg.FRONT_RIGHT= thrust_forces[3]
        tf_msg.FRONT_LEFT = thrust_forces[4]
        tf_msg.HEAVE_FRONT_LEFT = thrust_forces[5]
        tf_msg.HEAVE_BACK_LEFT = thrust_forces[6]
        tf_msg.BACK_LEFT = thrust_forces[7]
        
        # Publish the computed thruster forces (useful for simulation/debugging)
        self.pub_forces.publish(tf_msg)
        
        # Convert forces to PWM signals and publish them
        self.forces_to_pwm_publisher(thrust_forces)
    
    def forces_to_pwm_publisher(self, thrust_forces):
        """
        Converts thruster forces into PWM signals and publishes them.
        Applies individual limits to prevent overcurrent.
        """
        pwm_arr = [force_to_pwm_thruster(i+1, thrust_forces[i]) for i in range(8)]
    
        # Apply limit checking for each thruster
        pwm_arr = np.clip(pwm_arr,self.thruster_lower_limit, self.thruster_upper_limit)
        pwm_arr = pwm_arr.astype(np.uint16, copy=False)


        pwm_msg = ThrusterMicroseconds(pwm_arr.tolist())
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
