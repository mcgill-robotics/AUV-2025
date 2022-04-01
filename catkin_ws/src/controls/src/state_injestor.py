#!/usr/bin/env python3

import rospy

from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Pose
from tf.transformations import *

# set up PIDs for translation
# - x: setpoint_x, state_x, effort_x
# - y: setpoint_y, state_y, effort_y
# - z: setpoint_z, state_z, effort_z
state_x_pub = rospy.Publisher("state_x", Float64, queue_size=50)
state_y_pub = rospy.Publisher("state_y", Float64, queue_size=50)
state_z_pub = rospy.Publisher("state_z", Float64, queue_size=50)

# set up PID for rotation
# - the 'position variable' being controlled with the pid
# is the angle (theta) through which the robot is rotated around the 
# axis of rotation specified by sin(theta/2)(xi + yj + zk)
# where the orientation is described by quaternion <w, x, y, z>
# where w = cos(theta/2)
#
# - keep track of axis of rotation, during its rotation, should
# the robot deviate from rotating about this axis a new axis
# of rotation should be calculated that corresponds to a rotation
# that brings the robot into the target orientation. From this
# axis a quaternion can be constucted that describes the intended
# transformation. This would include updating the theta setpoint
# for the PID according to the new value of w.
#
# let the current orientation be described by q_1 (relative to datum)
# thus p_curr = q_1 * p_datum * q_1_inv => p_datum = q_1_inv * p_curr * q_1
#
# let the target orientation be described by q_2 (relative to datum)
# thus p_target = q_2 * p_datum * q_2_inv
# 
# a new quaternion decribing the transformation from the current orientation
# to the target orientation is found by:
# p_target = q_2 * (q_1_inv * p_curr * q_1) * q_2_inv
#
# thus q_3 = q_2 * q_1_inv
# 
#           q_1 = w_1 + x_1*i + y_1*j + z_1*k 
#               = cos(theta/2) + sin(theta/2)[x_1*i + y_1*j + z_1*k]
#   =>  q_1_inv = cos(-theta/2) + sin(-theta/2)[x_1*i + y_1*j + z_1*k]
#               = w_1 - x_1*i - y_1*j - z_1*k 

state_theta_pub = rospy.Publisher("state_theta", Float64, queue_size=50)

def state_cb(pose):
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z

    # quaternion 
    q_1_x = pose.orientation.x
    q_1_y = pose.orientation.y
    q_1_z = pose.orientation.z
    q_1_w = pose.orientation.w
    
    # update state for translation PIDs
    state_x_pub.publish(x)    
    state_y_pub.publish(y)    
    state_z_pub.publish(z)    

    # update state for rotation PID
    q_1_inv = [-q_1_x, -q_1_y, -q_1_z, q_1_w]
    q_2 = #TODO - get q_curr from Pursue_Target_Server
    q_3 = quaternion_multiply(q_2, q_1_inv)

    # TODO - update q_curr in Pursue_Target_Server
    theta = Float64(get_theta_from_q(q_3))
    state_theta_pub.publish(theta)

if __name__ == '__main__':
    rospy.init_node('state_injestor')
    rospy.Subscriber('state', Pose, state_cb, queue_size=50)
    rospy.spin()
