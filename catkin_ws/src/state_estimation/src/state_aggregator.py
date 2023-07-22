#!/usr/bin/env python3

import rospy
import numpy as np
import quaternion
from tf import transformations

from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from sbg_driver.msg import SbgEkfQuat
from std_msgs.msg import Empty, Float64
from auv_msgs.msg import DeadReckonReport
from sbg_driver.msg import SbgImuData


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

        # overall angular velocity # TODO - integrate
        self.angular_velocity = np.array([0.0, 0.0, 0.0])

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
        self.q_imu_mount_auv = np.quaternion(0, 0, 0, 1) # imu is rotated 180 degrees about z axis relative to AUV frame

        # publishers
        self.pub_auv = rospy.Publisher('pose', Pose, queue_size=1)
        self.pub_world = rospy.Publisher('pose_world', Pose, queue_size=1)

        self.pub_x = rospy.Publisher('state_x', Float64, queue_size=1)
        self.pub_y = rospy.Publisher('state_y', Float64, queue_size=1)
        self.pub_z = rospy.Publisher('state_z', Float64, queue_size=1)
        self.pub_theta_x = rospy.Publisher('state_theta_x', Float64, queue_size=1)
        self.pub_theta_y = rospy.Publisher('state_theta_y', Float64, queue_size=1)
        self.pub_theta_z = rospy.Publisher('state_theta_z', Float64, queue_size=1)
        self.pub_angular_velocity = rospy.Publisher('angular_velocity', Vector3, queue_size=1)

        # subscribers
        rospy.Subscriber("/sbg/ekf_quat", SbgEkfQuat, self.imu_cb)
        rospy.Subscriber("/dead_reckon_report",DeadReckonReport, self.dvl_cb)
        rospy.Subscriber("/depth", Float64, self.depth_sensor_cb)
        rospy.Subscriber("/reset_state_full", Empty, self.reset_state_full_cb)
        rospy.Subscriber("/reset_state_planar", Empty, self.reset_state_planar_cb)
        self.imu_sub = rospy.Subscriber("/sbg/imu_data", SbgImuData, self.set_imu)

        
    def set_imu(self, data):
        self.angular_velocity = np.array([data.gyro.x, data.gyro.y, data.gyro.z])


    def dvl_cb(self,data):
        # quaternion of DVL in NED frame
        q_dvl_ned = transformations.quaternion_from_euler(data.roll, data.pitch, data.yaw)
        q_dvl_ned = np.quaternion(q_dvl[3], q_dvl[0], q_dvl[1], q_dvl[2]) # transformations returns quaternion as nparray [w, x, y, z]


        # quaternion of AUV in global frame
        q_dvl_global = self.q_global_ned.inverse()*q_dvl_ned
        self.q_auv_global_dvl = q_dvl_global*self.q_dvl_mount_auv.inverse()

        # position of DVL (and also AUV TODO) in NED frame
        pos_dvl = np.array([data.x, data.y, data.z]) 

        # position of DVL in global frame
        self.pos_auv_global_dvl = quaternion.rotate_vectors(self.q_global_ned.inverse(), pos_dvl)


    def imu_cb(self, imu_msg):
        # quaternion of imu in NED frame
        q_imu_ned = imu_msg.quaternion
        q_imu_ned = np.quaternion(q_imu_ned.w, q_imu_ned.x, q_imu_ned.y, q_imu_ned.z) 

        # quaternion of AUV in global frame
        # sensor is mounted, may be oriented differently from AUV axes
        q_imu_global = self.q_global_ned.inverse()*q_imu_ned
        self.q_auv_global_imu = q_imu_global*self.q_imu_mount_auv.inverse()


    def depth_sensor_cb(self, depth_msg):
        # TODO - check does the 0 depth coincide with NED frame?
        # position of depth sensor (and assumed AUV TODO) in NED? frame
        pos_depth = np.array([0.0, 0.0, -depth_msg.data]) # TODO - depth is currrently lower (negative) - opposite of NED

        # TODO - compensate for sensor mounting position

        # position of depth sensor/AUV in global frame
        self.pos_auv_global_ds = quaternion.rotate_vectors(self.q_global_ned.inverse(), pos_depth)


    def reset_state_full_cb(self, _):
        '''
        Snaps the world frame to the AUV frame (position + orientation)
        '''
        # we can't trust self.q_auv_global or self.pos_auv_global
        # before it's been aggregated from sensors TODO - clarify which variables are reliable, refactor
        self.update_q_auv_world
        self.update_pos_auv_world
        # new world position is current AUV position in global frame
        self.pos_world_global = self.pos_auv_global
        self.pos_auv_world = np.array([0.0, 0.0, 0.0])

        # new world quaternion is q_auv in global frame
        self.q_world_global = self.q_auv_global

        # q_auv is still relative to old q_world, so is recalculated
        # by definition, it is aligned with the frame of q_world 
        # TODO - it is not necessary to update here, q_auv_world is always calculated
        # from q_auv_global before usage
        self.q_auv_world = np.quaternion(1, 0, 0, 0) 
        self.euler_auv_world = np.array([0.0, 0.0, 0.0])


    def reset_state_planar_cb(self, _):
        ''' 
        Snaps world frame to AUV position, 
        and similar orientation - except keeping the same (global) x-y plane 

        - this method is a bit hackish 
        and should only be used when the orientation of the AUV 
        is aproximately only yaw relative to global frame, 
        it may give unwanted results at other extreme orientations
        '''
        # we can't trust self.q_auv_global or self.pos_auv_global
        # before it's been aggregated from sensors TODO - clarify which variables are reliable, refactor
        self.update_q_auv_world
        self.update_pos_auv_world
        # new world position is x, y - current AUV position, z - same as previous world frame, in global frame
        self.pos_world_global[0:2] = self.pos_auv_global[0:2] # do not change z
        # update AUV position relative to new world frame
        # TODO - is this nececssary? pos_auv_world is always calculated before publishing
        pos_auv = self.pos_auv_global - self.pos_world_global
        self.pos_auv_world = quaternion.rotate_vectors(self.q_world_global.inverse(), pos_auv)

        # new world quaternion is yaw rotation of AUV in global frame
        q = self.q_auv_global
        q_auv_global = np.array([q.x, q.y, q.z, q.w])
        theta_z = transformations.euler_from_quaternion(q_auv_global, 'szyx')[0]
        q_world_global = transformations.quaternion_from_euler(0, 0, theta_z)
        self.q_world_global = np.quaternion(q_world_global[3], q_world_global[0], q_world_global[1], q_world_global[2])

        # q_auv is still relative to old world frame, so is recalculated
        # note - the new AUV frame does not necessarily coincide with the 
        # new world frame since we are using only the yaw component of 
        # q_auv_global when determining the world frame
        self.q_auv_world = self.q_world_global.inverse()*self.q_auv_global

        # euler is reset to erase any windup, then is brought to
        # the actual euler angle of the AUV
        self.euler_auv_world = np.array([0.0, 0.0, 0.0])
        self.update_euler()


    def update_q_auv_world(self):
        # currently only using the imu for orientation
        self.q_auv_global = self.q_auv_global_imu
        # TODO - instead of having auv_global in attributes, it should only be 
        # available as getter method which calculates it from sensors

        # quaternion of AUV in world frame
        self.q_auv_world = self.q_world_global.inverse()*self.q_auv_global
    

    def update_pos_auv_world(self):
        # aggregate AUV position in global frame
        self.pos_auv_global = np.array([
            self.pos_auv_global_dvl[0], 
            self.pos_auv_global_dvl[1], 
            self.pos_auv_global_ds[2]])

        # AUV position/offset from pos_world (vector as seen in global frame)
        pos_auv = self.pos_auv_global - self.pos_world_global

        # position in world frame
        self.pos_auv_world = quaternion.rotate_vectors(self.q_world_global.inverse(), pos_auv)


    def update_euler(self):
        # calculate euler angles based on self.q_auv_world
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
        position = Point(x=self.pos_auv_world[0], y=self.pos_auv_world[1], z=self.pos_auv_world[2]) # TODO - check nparray parsed - may put directly into Pose
        orientation = Quaternion(x=self.q_auv_world.x, y=self.q_auv_world.y, z=self.q_auv_world.z, w=self.q_auv_world.w)
        pose = Pose(position, orientation)
        self.pub_auv.publish(pose)

        # publish world pose
        position = Point(x=self.pos_world_global[0], y=self.pos_world_global[1], z=self.pos_world_global[2]) # TODO - check nparray parsed - may put directly into Pose
        orientation = Quaternion(x=self.q_world_global.x, y=self.q_world_global.y, z=self.q_world_global.z, w=self.q_world_global.w)
        pose = Pose(position, orientation)
        self.pub_world.publish(pose)

        # publish individual degrees of freedom
        self.pub_x.publish(self.pos_auv_world[0])
        self.pub_y.publish(self.pos_auv_world[1])
        self.pub_z.publish(self.pos_auv_world[2])

        self.pub_theta_x.publish(self.euler_auv_world[0])
        self.pub_theta_y.publish(self.euler_auv_world[1])
        self.pub_theta_z.publish(self.euler_auv_world[2])

        angular_velocity = Vector3(self.angular_velocity[0], self.angular_velocity[1], self.angular_velocity[2])
        self.pub_angular_velocity.publish(angular_velocity)



if __name__ == '__main__':
    rospy.init_node('state_aggregator')
    sa = State_Aggregator()
    timer = rospy.Timer(rospy.Duration(0.1), sa.update_state)
    rospy.on_shutdown(timer.shutdown)
    rospy.spin()
