#!/usr/bin/env python3

import rospy
import tf
import math
import numpy as np
import quaternion

from geometry_msgs.msg import Point, Pose, Quaternion
from sbg_driver.msg import SbgEkfQuat
from state_variables import *
from std_msgs.msg import Empty, Float64


# angles that change by more than 90 degrees between readings 
# are assumed to wrap around
ANGLE_CHANGE_TOL = 90 

DEG_PER_RAD = 180/math.pi

class State_Aggregator:

    def __init__(self):
        # position
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # orientation
        self.quat = np.quaternion(1, 0, 0, 0) # w, x, y, z - relative to quat_ref
        self.quat_ref = np.quaternion(1, 0, 0, 0) # nominal orientation - allows imu reset
        self.euler = np.array([0.0, 0.0, 0.0]) # to determine when wrap-around happens 

        # publishers
        self.pub = rospy.Publisher('pose', Pose, queue_size=50)
        self.pub_x = rospy.Publisher('state_x', Float64, queue_size=50)
        self.pub_y = rospy.Publisher('state_y', Float64, queue_size=50)
        self.pub_z = rospy.Publisher('state_z', Float64, queue_size=50)
        self.pub_theta_x = rospy.Publisher('state_theta_x', Float64, queue_size=50)
        self.pub_theta_y = rospy.Publisher('state_theta_y', Float64, queue_size=50)
        self.pub_theta_z = rospy.Publisher('state_theta_z', Float64, queue_size=50)

        # subscribers
        rospy.Subscriber("/sbg/ekf_quat", SbgEkfQuat, self.imu_cb)
        rospy.Subscriber("/depth", Float64, self.depth_sensor_cb)
        rospy.Subscriber("imu_reset", Empty, self.imu_reset_cb)


    def imu_cb(self, imu_msg):
        q = imu_msg.quaternion
        self.quat = np.quaternion(q.w, q.x, q.y, q.z)*self.quat_ref.inverse()

        # calculate euler angles
        angles = quaternion.as_euler_angles(self.quat)*DEG_PER_RAD

        # allow angles to wind up to preserve continuity
        for i in range(3):
            if angles[i] - self.euler[i] > ANGLE_CHANGE_TOL:
                self.euler[i] = angles[i] - 360
            elif self.euler[i] - angles[i] > ANGLE_CHANGE_TOL:
                self.euler[i] = angles[i] + 360
            else:
                self.euler[i] = angles[i]


    def depth_sensor_cb(self, depth_msg):
        # TODO - have depth microcontroller publish ready value
        self.z = depth_msg.data*-1


    def update_state(self, _):

        # publish pose
        position = Point(self.x, self.y, self.z)
       	pose  = Pose(position, self.quat)
        self.pub.publish(pose)


        # publish individual degrees of freedom
        self.pub_x.publish(self.x)
        self.pub_y.publish(self.y)
        self.pub_z.publish(self.z)

        self.pub_theta_x.publish(self.euler[0])
        self.pub_theta_y.publish(self.euler[1])
        self.pub_theta_z.publish(self.euler[2])


    def imu_reset_cb(self, _):
        # this will only propogate on the next imu_cb 
        # q = self.quat
        # new reference rotation is compound of current reference rotation 
        # and the current rotation (wrt current reference rotation) 
        # this is same as getting the reference rotation relative 
        # to a global frame of the original imu quaternion readings
        # self.quat_ref = np.quaternion(q.w, q.x, q.y, q.z)*self.quat_ref


if __name__ == '__main__':
    rospy.init_node('state_aggregator')
    sa = State_Aggregator()
    timer = rospy.Timer(rospy.Duration(0.1), sa.update_state)
    rospy.on_shutdown(timer.shutdown)
    rospy.spin()
