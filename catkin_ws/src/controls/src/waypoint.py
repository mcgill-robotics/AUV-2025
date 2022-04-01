#!/usr/bin/env python3

import actionlib
import rospy

from auv_msgs.msg import WaypointAction, WaypointFeedback, WaypointResult
from geometry_msgs.msg import Pose, Quaternion
from state import Setpoint_State, Perceived_State
from std_msgs.msg import Float64, Bool
from tf import quaternion_multiply

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

# values to reuse

enable = Bool(True)
disable = Bool(False)

class Waypoint_Server:
    epsilon_x = epsilon_y = epsilon_z = 0.05    # permissible error [m]
    epsilon_theta = 0.05                        # permissible error [rad]

    def __init__(self):
        self.curr_state = None
        self.target_state = None
        self.axis_of_rotation = None

        self.pid_enable_pub = rospy.Publisher(
                'enable', Bool, queue_size=50)
        rospy.Subscriber('state', Pose, self.state_cb, queue_size=50)
        self.server = actionlib.SimpleActionServer(
                'waypoint', WaypointAction, self.execute_cb, False)


    '''
    acceptable region is box - may be changed later
    '''
    def is_target_reached(self):
        dx = abs(self.target_state.position.x - self.curr_state.position.x)
        dy = abs(self.target_state.position.y - self.curr_state.position.y)
        dz = abs(self.target_state.position.z - self.curr_state.position.z)
        dtheta = abs(self.target_state.orientation.theta 
                    - self.curr_state.orientation.theta)

        return dx <= Waypoint_Server.epsilon_x 
                and dy <= Waypoint_Server.epsilon_y
                and dz <= Waypoint_Server.epsilon_z
                and dtheta <= Waypoint_Server.epsilon_theta


    def state_cb(self, pose):
        self.curr_state = Perceived_State(pose)
        
        # make sure state update is atomic
        self.pid_enable_pub.publish(disable)
        self.curr_state.publish()
        if self.target_state is not None:
            self.target_state.publish_angle_diff(self.curr_state)
        self.pid_enable_pub.publish(enable)
    
    def feedback_cb(self, _):
        curr_pose = self.curr_state.as_pose() 
        feedback = WaypointFeedback(curr_pose)
        self.server.publish_feedback(feedback)


    def execute_cb(self, goal):
        self.target_state = goal

        # make sure target update is atomic
        self.pid_enable_pub.publish(disable)
        self.target_state.publish()
        if self.curr_state is not None:
            self.target_state.publish_angle_diff(self.curr_state)
        self.pid_enable_pub.publish(enable)
        
        # give feedback to action client
        feedback_timer = rospy.Timer(rospy.Duration(0.1), self.feedback_cb)
        
        while not self.is_target_reached():
            continue
        
        feedback_timer.shutdown()
        self.target_state = None
        
        final_pose = self.curr_state.as_pose() 
        result = WaypointResult(final_pose)
        self.server.set_succeeded(result)


if __name__ == '__main__':
    rospy.init_node('waypoint_server')
    ws = Waypoint_Server()
    ws.server.start()
    rospy.spin()

