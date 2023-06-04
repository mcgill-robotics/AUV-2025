#!/usr/bin/env python3

import rospy
import numpy as np
import quaternion
from tf import transformations

from geometry_msgs.msg import Point, Pose, Quaternion 
from sbg_driver.msg import SbgEkfQuat
from std_msgs.msg import Empty, Float64
from auv_msgs.msg import DeadReckonReport


# angles that change by more than 90 degrees between readings 
# are assumed to wrap around
ANGLE_CHANGE_TOL = 90 

DEG_PER_RAD = 180/np.pi

class State_Aggregator:

    def __init__(self):
        # position
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # orientation
        self.q_auv = np.quaternion(1, 0, 0, 0) # w, x, y, z - orientation of AUV as seen from world frame
        self.q_world = np.quaternion(0, 1, 0, 0) # 'nominal' orientation, relative to global (imu) frame - allows imu reset 
        self.euler = np.array([0.0, 0.0, 0.0]) # to determine when wrap-around happens, tracks q_auv

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
        rospy.Subscriber("dead_reckon_report",DeadReckonReport, self.dr_cb)

        '''
        TODO - use tf2 instead of publishing pose?
        # transform broadcasters
        self.br_world = TransformBroadcaster()
        self.br_auv_base = StaticTransformBroadcaster()
        
        # transform messages
        self.tf_world = TransformStamped()
        self.tf_auv_base = TransformStamped()
        '''

    def dr_cb(self,data):
        #THIS IS TEMPORARY
        self.x = data.x
        self.y = data.y

    def imu_cb(self, imu_msg):
        # quaternion in global (imu) frame
        q_imu = imu_msg.quaternion
        q_imu = np.quaternion(q_imu.w, q_imu.x, q_imu.y, q_imu.z) 

        # quaternion in world frame
        self.q_auv= self.q_world.inverse()*q_imu
        self.update_euler()


    def depth_sensor_cb(self, depth_msg):
        # TODO - have depth microcontroller publish ready value
        self.z = depth_msg.data

    def imu_reset_cb(self, _):
        # copy of q_auv in (old) world frame
        q_auv = self.q_auv
        q_auv = np.quaternion(q_auv.w, q_auv.x, q_auv.y, q_auv.z)

        # new world quaternion is q_auv in global (imu) frame
        self.q_world = self.q_world.inverse()*q_auv

        # q_auv is still relative to old q_world, so is recalculated
        # by definition, it is aligned with the frame of q_world 
        self.q_auv = np.quaternion(1, 0, 0, 0) 
        self.euler = np.array([0.0, 0.0, 0.0])

    def update_euler(self):
        # calculate euler angles
        # *note* the tf.transformations module is used instead of
        # quaternion package because it gives the euler angles
        # as roll/pitch/yaw (XYZ) whereas the latter chooses
        # a different euler angle convention (ZYZ)
        # tf.transformations uses quaternion defined as [x, y, z, w]
        # whereas quaternion is ordered [w, x, y, z]
        q = self.q_auv
        q = np.array([q.x, q.y, q.z, q.w])
        
        # rotations are applied to ‘s’tatic or ‘r’otating frame
        # we're just getting the first angle - this assumes
        # that the other angles are fixed 
        # TODO - these angles don't combine, only to be treated individually
        theta_x = transformations.euler_from_quaternion(q, 'rxyz')[0]
        theta_y = transformations.euler_from_quaternion(q, 'ryxz')[0]
        theta_z = transformations.euler_from_quaternion(q, 'rzyx')[0]

        # euler_from_quaternion returns 3-tuple of radians
        # convert to numpy array of degrees
        angles = np.array([theta_x, theta_y, theta_z])*DEG_PER_RAD

        # allow angles to wind up to preserve continuity
        for i in range(3):
            if angles[i] - self.euler[i] > ANGLE_CHANGE_TOL:
                self.euler[i] = angles[i] - 360
            elif self.euler[i] - angles[i] > ANGLE_CHANGE_TOL:
                self.euler[i] = angles[i] + 360
            else:
                self.euler[i] = angles[i]


    def update_state(self, _):
        # publish pose
        position = Point(self.x, self.y, self.z)
        pose = Pose(position, self.q_auv)
        self.pub.publish(pose)

        # publish individual degrees of freedom
        self.pub_x.publish(self.x)
        self.pub_y.publish(self.y)
        self.pub_z.publish(self.z)

        self.pub_theta_x.publish(self.euler[0])
        self.pub_theta_y.publish(self.euler[1])
        self.pub_theta_z.publish(self.euler[2])



if __name__ == '__main__':
    rospy.init_node('state_aggregator')
    sa = State_Aggregator()
    timer = rospy.Timer(rospy.Duration(0.1), sa.update_state)
    rospy.on_shutdown(timer.shutdown)
    rospy.spin()
