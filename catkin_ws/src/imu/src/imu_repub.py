#!/usr/bin/env python3

import rospy
import tf
import numpy as np

from geometry_msgs.msg import Quaternion
from auv_msgs.msg import ImuData
from sbg_driver.msg import SbgEkfQuat


pub = rospy.Publisher('imu_data', ImuData, queue_size=50)

def sbg_imu_cb(msg):
    q = msg.quaternion
    q_arr = [q.x, q.y, q.z, q.w]
    euler = tf.transformations.euler_from_quaternion(q_arr)

    euler_msg = ImuData(180/np.pi*euler[0], 180/np.pi*euler[1], 180/np.pi*euler[2])
    pub.publish(euler_msg)
    return


if __name__ == '__main__':
    rospy.init_node('imu_repub')
    rospy.Subscriber("/sbg/ekf_quat", SbgEkfQuat, sbg_imu_cb)
    rospy.spin()
