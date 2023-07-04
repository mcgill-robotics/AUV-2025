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
        '''Overall state'''
        # overall position (x,y - dvl, z - depth sensor)
        self.pos_world_global = np.array([0.0, 0.0, 0.0])
        self.pos_auv_world = np.array([0.0, 0.0, 0.0])
        self.pos_auv_global = np.array([0.0, 0.0, 0.0])

        # overall orientation (imu)
        self.q_global_ned = np.quaternion(0, 1, 0, 0) # orientation of inertial global frame relative to North-East-Down (we use NWU convention)
        self.q_world_global = np.quaternion(1, 0, 0, 0) # orientation of world frame relative to global frame
        self.q_auv_global = np.quaternion(1, 0, 0, 0) # orientation of AUV in global frame
        self.q_auv_world = np.quaternion(1, 0, 0, 0) # orientation of AUV in world frame - updated before publication
        self.euler_auv_world = np.array([0.0, 0.0, 0.0]) # to determine when wrap-around happens, tracks q_auv_world

        '''DVL'''
        self.pos_auv_global_dvl = np.array([0.0, 0.0, 0.0]) # position of AUV relative to global frame - according to DVL
        self.q_auv_global_dvl = np.quaternion(1, 0, 0, 0) # w, x, y, z - orientation of AUV relative to global frame 
        self.q_dvl_mount_auv = np.quaternion(1, 0, 0, 0) # initial orientation of DVL in AUV frame (measured/assumed based on mounting of sensor) #TODO
        self.pos_dvl_mount_auv = np.array([0.0, 0.0, 0.0]) # initial position of DVL in AUV frame (measured/assumed based on mounting of sensor) #TODO

        '''Depth Sensor'''
        self.pos_auv_global_ds = np.array([0.0, 0.0, 0.0])
        self.q_ds_mount_auv = np.quaternion(1, 0, 0, 0) 
        self.pos_ds_mount_auv = np.array([0.0, 0.0, 0.0])

        '''IMU'''
        self.q_auv_global_imu = np.quaternion(1, 0, 0, 0) 
        self.q_imu_mount_auv = np.quaternion(0, 1, 0, 0) 

        # publishers
        self.pub_auv = rospy.Publisher('pose', Pose, queue_size=1)
        self.pub_world = rospy.Publisher('pose_world', Pose, queue_size=1)
        self.pub_x = rospy.Publisher('state_x', Float64, queue_size=1)
        self.pub_y = rospy.Publisher('state_y', Float64, queue_size=1)
        self.pub_z = rospy.Publisher('state_z', Float64, queue_size=1)
        self.pub_theta_x = rospy.Publisher('state_theta_x', Float64, queue_size=1)
        self.pub_theta_y = rospy.Publisher('state_theta_y', Float64, queue_size=1)
        self.pub_theta_z = rospy.Publisher('state_theta_z', Float64, queue_size=1)

        # subscribers
        rospy.Subscriber("/sbg/ekf_quat", SbgEkfQuat, self.imu_cb)
        rospy.Subscriber("/dead_reckon_report",DeadReckonReport, self.dvl_cb)
        rospy.Subscriber("/depth", Float64, self.depth_sensor_cb)
        rospy.Subscriber("/reset_state", Empty, self.reset_state_cb)


    def dvl_cb(self,data):
        # quaternion of DVL in NED frame
        q_dvl = transformations.quaternion_from_euler(data.roll, data.pitch, data.yaw)
        q_dvl = np.quaternion(q_dvl[3], q_dvl[0], q_dvl[1], q_dvl[2]) # transformations returns quaternion as nparray [w, x, y, z]

        # quaternion of AUV in global frame
        q_auv_global_dvl = self.q_dvl_mount_auv.inverse()*self.q_global_ned.inverse()*q_dvl

        # position of DVL (and also AUV TODO) in NED frame
        pos_dvl = np.array([data.x, data.y, data.z]) 

        # position of DVL in global frame
        pos_auv_global_dvl = quaternion.rotate_vectors(self.q_global_ned.inverse(), pos_dvl)


    def imu_cb(self, imu_msg):
        # quaternion of imu in NED frame
        q_imu = imu_msg.quaternion
        q_imu = np.quaternion(q_imu.w, q_imu.x, q_imu.y, q_imu.z) 

        # quaternion of AUV in global frame
        # sensor is mounted, may be oriented differently from AUV axes
        self.q_auv_global_imu = self.q_imu_mount_auv.inverse()*self.q_global_ned*q_imu


    def depth_sensor_cb(self, depth_msg):
        # TODO - check does the 0 depth coincide with NED frame?
        # position of depth sensor (and assumed AUV TODO) in NED? frame
        pos_depth = np.array([0.0, 0.0, -depth_msg.data]) # TODO - depth is currrently lower (negative) - opposite of NED

        # TODO - compensate for sensor mounting position

        # position of depth sensor/AUV in global frame
        self.pos_auv_global_ds = quaternion.rotate_vectors(self.q_global_ned.inverse(), pos_depth)


    def reset_state_cb(self, _):
        # TODO - for now, only reset the orientation
        # maybe on reset find best world frame that keeps the same x-y plane?
        # new world position is current AUV position in global frame
        # self.pos_world = self.pos_auv + self.pos_world
        # self.pos_auv = np.array([0.0, 0.0, 0.0])

        # copy of q_auv in (old) world frame
        q_auv = self.q_auv_world
        q_auv = np.quaternion(q_auv.w, q_auv.x, q_auv.y, q_auv.z)

        # new world quaternion is q_auv in global (imu) frame
        self.q_world_global = self.q_world_global*q_auv

        # q_auv is still relative to old q_world, so is recalculated
        # by definition, it is aligned with the frame of q_world 
        self.q_auv_world = np.quaternion(1, 0, 0, 0) 
        self.euler_auv_world = np.array([0.0, 0.0, 0.0]) # TODO - calculate at publish


    def update_q_auv_world(self):
        # currently only using the imu for orientation
        q_auv_global = self.q_auv_global_imu

        # quaternion of AUV in world frame
        self.q_auv_world = self.q_world_global.inverse()*self.q_auv_global
    

    def update_pos_auv_world(self):
        # aggregate AUV position in global frame
        pos_auv_global = np.array([
            self.pos_auv_global_dvl[0], 
            self.pos_auv_global_dvl[1], 
            self.pos_auv_global_ds[2]])

        # AUV position/offset from pos_world (vector as seen in global frame)
        pos_auv = self.pos_auv_global - self.pos_world_global

        # position in world frame
        self.pos_auv_world = quaternion.rotate_vectors(self.q_world_global.inverse(), pos_auv)


    def update_euler(self):
        # calculate euler angles
        # *note* the tf.transformations module is used instead of
        # quaternion package because it gives the euler angles
        # as roll/pitch/yaw (XYZ) whereas the latter chooses
        # a different euler angle convention (ZYZ)
        # tf.transformations uses quaternion defined as [x, y, z, w]
        # whereas quaternion is ordered [w, x, y, z]
        q = self.q_auv_world
        q = np.array([q.x, q.y, q.z, q.w])
        
        # rotations are applied to ‘s'tatic or ‘r’otating frame
        # we're just getting the first angle - this assumes
        # that the other angles are fixed 
        # TODO - these angles don't combine, only to be treated individually
        # theta = transformations.euler_from_quaternion(q, 'rxyz')
        # theta_x = theta[0]
        # theta_y = theta[1]
        # theta_z = theta[2]
        theta_x = transformations.euler_from_quaternion(q, 'rxyz')[0]
        theta_y = transformations.euler_from_quaternion(q, 'ryxz')[0]
        theta_z = transformations.euler_from_quaternion(q, 'rzyx')[0]

        # euler_from_quaternion returns 3-tuple of radians
        # convert to numpy array of degrees
        angles = np.array([theta_x, theta_y, theta_z])*DEG_PER_RAD

        # allow angles to wind up to preserve continuity
        for i in range(3):
            if angles[i] - self.euler_auv_world[i] > ANGLE_CHANGE_TOL:
                self.euler_auv_world[i] = angles[i] - 360
            elif self.euler_auv_world[i] - angles[i] > ANGLE_CHANGE_TOL:
                self.euler_auv_world[i] = angles[i] + 360
            else:
                self.euler_auv_world[i] = angles[i]


    def update_state(self, _):
        # update state of AUV in world frame
        self.update_q_auv_world()
        self.update_pos_auv_world()
        self.update_euler()

        # publish AUV pose
        position = Point(self.pos_auv_world[0], self.pos_auv_world[1], self.pos_auv_world[2]) # TODO - check nparray parsed - may put directly into Pose
        pose = Pose(position, self.q_auv_world)
        self.pub_auv.publish(pose)

        # publish world pose
        position = Point(self.pos_world_global[0], self.pos_world_global[1], self.pos_world_global[2]) # TODO - check nparray parsed - may put directly into Pose
        pose = Pose(position, self.q_world_global)
        self.pub_world.publish(pose)

        # publish individual degrees of freedom
        self.pub_x.publish(self.pos_auv_world[0])
        self.pub_y.publish(self.pos_auv_world[1])
        self.pub_z.publish(self.pos_auv_world[2])

        self.pub_theta_x.publish(self.euler_auv_world[0])
        self.pub_theta_y.publish(self.euler_auv_world[1])
        self.pub_theta_z.publish(self.euler_auv_world[2])



if __name__ == '__main__':
    rospy.init_node('state_aggregator')
    sa = State_Aggregator()
    timer = rospy.Timer(rospy.Duration(0.1), sa.update_state)
    rospy.on_shutdown(timer.shutdown)
    rospy.spin()
