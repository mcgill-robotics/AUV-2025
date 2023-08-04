#!/usr/bin/env python3

import rospy
import numpy as np
import quaternion
from tf import transformations

from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from sensors import DepthSensor, IMUSensor, DVLSensor
from std_msgs.msg import Empty, Float64

DEG_PER_RAD = 180/np.pi

# TODO - update conventions
'''
Note on variable naming conventions used in this file:

Broadly speaking, a name indicates what frame is being measured
as well as relative to what frame the measurement is done. 

<type>_<target_frame>_<reference_frame>

Rotations:
    Ex: q_auv_global should be read as:
        "a quaternion representing the AUV frame relative to the
        global frame" 
            or 
        "a quaternion that takes the basis vectors of the global frame
        into the basis vectors of the AUV frame"

    the same rotation can be expressed in euler angles
    or as a quaternion:

    q_auv_global = np.quaternion(0.707, 0.707, 0, 0) # w, x, y, z
    euler_auv_global = np.array([180, 0, 0]) # roll/pitch/yaw in degrees

Positions:
    Ex: pos_dvl_dvlref should be read as:
        "the position of dvl frame according to the dvlref frame"
            or
        "a vector emanating from the origin of dvlref frame, pointing
        to the origin of dvl frame, as seen in the dvlref frame"

    If a vector is described as: pos_<frame1>_<frame2>_<frame3>
    the (origins of) first two frames represent the endpoint and 
    start-point of the vector, <frame3> represents the frame from which 
    this vector is viewed

    Ex: pos_dvl_dvlref_global should be read as:
        "a vector emanating from the origin of dvlref frame, pointing
        to the origin of dvl frame, as seen in the global frame"

    *Notice, a variable described as pos_dvl_dvlref is implicitly 
    viewed in dvlref frame. 

Mounting positions/rotations are described similarly, to 
highlight that these entities are static based on the mounting 
of sensors on the auv the postfix '_mount' is used after
<target_frame>

    Ex: pos_dvl_mount_auv should be read as:
    "The position vector from AUV frame to dvl frame
    (viewed from AUV frame)"
    here, 'mount' is not a reference frame

We also keep track of q_auv_global/pos_auv_global
according to several sensors, so the position
according to the depth sensor data, and imu data are stored in:
    pos_auv_global__fromds
    pos_auv_global__fromimu

This is done to disambiguate from pos_auv_global_dvl
which could b# np.quaternion(1, 0, 0, 0)e taken to mean "vector from
global to auv, as seen in dvl frame"

'''


class State_Aggregator:
    def __init__(self):
        # world frame relative to global
        self.pos_world_global = np.array([0.0, 0.0, 0.0])
        self.q_world_global = np.quaternion(1, 0, 0, 0)

        # redundancy - without previous messages, assume default
        self.last_pos_auv_global = np.zeros(3)
        self.last_q_auv_global = np.quaternion(1, 0, 0, 0)
        self.last_w_auv = np.zeros(3)

        # sensors
        # TODO - pass in mount q/pos as contructor args
        self.depth_sensor = DepthSensor()
        self.imu = IMUSensor()
        self.dvl = DVLSensor(self.set_dvlref)

        # publishers
        self.pub_auv = rospy.Publisher('pose', Pose, queue_size=1)
        self.pub_world = rospy.Publisher('pose_world', Pose, queue_size=1)
        self.pub_x = rospy.Publisher('state_x', Float64, queue_size=1)
        self.pub_y = rospy.Publisher('state_y', Float64, queue_size=1)
        self.pub_z = rospy.Publisher('state_z', Float64, queue_size=1)
        self.pub_theta_x = rospy.Publisher('state_theta_x', Float64, queue_size=1)
        self.pub_theta_y = rospy.Publisher('state_theta_y', Float64, queue_size=1)
        self.pub_theta_z = rospy.Publisher('state_theta_z', Float64, queue_size=1)
        self.pub_w_auv = rospy.Publisher('angular_velocity', Vector3, queue_size=1)

        # subscribers
        rospy.Subscriber("/reset_state_full", Empty, self.reset_state_full_cb)
        rospy.Subscriber("/reset_state_planar", Empty, self.reset_state_planar_cb)



    '''
    The methods pos_auv_global, pos_auv_world, q_auv_global, q_auv_world, (and euler_auv_world)
    calculate these entities based on the current available sensor data. 

    Tracking these values as attributes may introduce bugs: ie. pos_world_global 
    is updated but not pos_auv_world leading to logic errors.

    This is avoided by recalculating pos_auv_world etc. any time it is needed
    '''

    def pos_auv_global(self):
        pos_auv_global = self.last_pos_auv_global # latch last in case sensors are inactive

        # determine xy
        if self.dvl.is_active:
            pos_auv_global[0:2] = self.dvl.pos_auv_global(self.q_auv_global())[0:2]
        else:
            rospy.logwarn("state_aggregator - dvl inactive, unable to fully resolve AUV pose (xy), /pose is unreliable")

        # determine z
        if self.depth_sensor.is_active:
            pos_auv_global[2] = self.depth_sensor.pos_auv_global(self.q_auv_global())[2]
        # TODO - check continuity
        elif self.dvl.is_active:
            pos_auv_global[2] = self.dvl.pos_auv_global(self.q_auv_global())[2]
        else:
            rospy.logwarn("state_aggregator - dvl/depth sensor inactive, unable to fully resolve AUV pose (z), /pose is unreliable")
            # TODO - what to return when not possible to resolve state?

        self.last_pos_auv_global = pos_auv_global
        return pos_auv_global 


    def q_auv_global(self):
        q_auv_global = self.last_q_auv_global # latch last in case sensors are inactive

        if self.imu.is_active:
            q_auv_global =  self.imu.q_auv_global()
        elif self.dvl.is_active:
            q_auv_global =  self.dvl.q_auv_global()
        else:
            rospy.logwarn("state_aggregator - dvl/imu inactive, unable to fully resolve AUV pose (orientation), /pose is unreliable")

        self.last_q_auv_global = q_auv_global
        return q_auv_global


    def w_auv(self):
        w_auv = self.last_w_auv
        
        if self.imu.is_active:
            w_auv = self.imu.w_auv()
        else:
            # TODO - use dvl for w_auv readings
            # TODO - return last w_auv or something else (ie [0, 0, 0]) for safety?
            rospy.logwarn("state_aggregator - imu inactive, unable to resolve AUV angular velocity, /angular_velocity is unreliable")

        return w_auv



    def pos_auv_world(self):
        # AUV position/offset from pos_world (vector as seen in global frame)
        pos_auv_world_global = self.pos_auv_global() - self.pos_world_global

        # position in world frame
        pos_auv_world = quaternion.rotate_vectors(self.q_world_global.inverse(), pos_auv_world_global)
        return pos_auv_world


    def q_auv_world(self):
        q_auv_world = self.q_world_global.inverse()*self.q_auv_global()
        return q_auv_world 
        

    def euler_auv_world(self):
        '''
        The use of euler angles is for backward compatibility
        to publish data to state_theta_* topics
        '''
        if self.q_auv_world() is None:
            return None

        # calculate euler angles based on self.q_auv_world
        # *note* the tf.transformations module is used instead of
        # quaternion package because it gives the euler angles
        # as roll/pitch/yaw (XYZ) whereas the latter chooses
        # a different euler angle convention (ZYZ)
        # tf.transformations uses quaternion defined as [x, y, z, w]
        # whereas quaternion is ordered [w, x, y, z]
        q = self.q_auv_world()
        q = np.array([q.x, q.y, q.z, q.w])
        
        # rotations are applied to ‘s'tatic or ‘r’otating frame
        # we're just getting the first angle - this assumes
        # that the other angles are fixed 
        # these angles don't combine, only to be treated individually
        # theta = transformations.euler_from_quaternion(q, 'rxyz')
        # theta_x = theta[0]
        # theta_y = theta[1]
        # theta_z = theta[2]
        theta_x = transformations.euler_from_quaternion(q, 'rxyz')[0]
        theta_y = transformations.euler_from_quaternion(q, 'ryxz')[0]
        theta_z = transformations.euler_from_quaternion(q, 'rzyx')[0]

        # euler_from_quaternion returns 3-tuple of radians
        # convert to numpy array of degrees
        euler_auv_world = np.array([theta_x, theta_y, theta_z])*DEG_PER_RAD
        return euler_auv_world


    '''
    Methods to alter the world frame relative to the global frame. 

    The world frame for-all-intents-and-purposes can be treated as the global (inertial) 
    frame. However, it may be useful to change the world frame at the beginning of the 
    run to align the axes with the geometry of the pool, this makes it more clear which 
    direction are the x/y axes

    It may be a bit dangerous changing the world frame during the run 
    as this might fuck with what the PIDs are doing - care is required
    '''

    def reset_state_full_cb(self, _):
        '''
        Snaps the world frame to the AUV frame (position + orientation)
        '''
        # TODO - cannot reset if auv_global vars not set
        # new world position is current AUV position in global frame
        self.pos_world_global = self.pos_auv_global()

        # new world quaternion is q_auv in global frame
        self.q_world_global = self.q_auv_global()


    def reset_state_planar_cb(self, _):
        ''' 
        Snaps world frame to AUV x/y position (z constant)
        and similar orientation - except keeping the same (global) x-y plane 

        - this method is a bit hackish 
        and should only be used when the orientation of the AUV 
        is aproximately only yaw relative to global frame, 
        it may give unwanted results at other extreme orientations
        '''
        # new world position is x, y - current AUV position, z - same as previous world frame, in global frame
        self.pos_world_global[0:2] = self.pos_auv_global()[0:2] # do not change z

        # new world quaternion is yaw rotation of AUV in global frame
        q = self.q_auv_global()
        q_auv_global = np.array([q.x, q.y, q.z, q.w])
        theta_z = transformations.euler_from_quaternion(q_auv_global, 'szyx')[2]
        q_world_global = transformations.quaternion_from_euler(0, 0, theta_z)
        self.q_world_global = np.quaternion(q_world_global[3], q_world_global[0], q_world_global[1], q_world_global[2])
        print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")


    def set_dvlref(self):
        # TODO - check q_auv_global is available
        if self.pos_auv_global() is None:
            pos_auv_global_init = np.array([0, 0, 0])
            self.dvl.set_dvlref_global(self.q_auv_global(), pos_auv_global_init)
        else:
            self.dvl.set_dvlref_global(self.q_auv_global(), self.pos_auv_global())

        

    '''
    Initialization makes sure all the relevant reference frames are well defined
    before trying to do (potentially ambiguous) frame transformations

    This method should always be called prior to trying to reset the world frame
    or publish state
    '''

    def initialize(self):
        rospy.loginfo("state_aggregator initializing -- waiting on sensor data")
        # TODO - does this update state while blocked? could be issue using old q_auv_global
        # TODO - do not block if some sensors remain inactive 
        # wait for data from imu
        while not self.imu.is_active:
            pass

        rospy.loginfo("state_aggregator initializing -- imu active, waiting on depth_sensor")

        '''
        # TODO - if a_auv_global relies on other sensors, make sure they are also initialized
        # wait for data from depth sensor
        while not self.depth_sensor.is_active: 
            pass

        rospy.loginfo("state_aggregator initializing -- depth_sensor active, waiting on dvl")
        '''

        # set dvlref frame which is reference to dvl readings, 
        # xy are arbitrarily set to whatever depth_sensor thinks
        # (this may be something other than 0, 0 after accounting for mounting location)
        # wait for dvl data (without which we can't set dvlref)
        pos_auv_global_init = np.array([0, 0, 0])
        while not self.dvl.is_active:
            try:
                self.set_dvlref()
            except:
                rospy.sleep(1) # TODO - doesn't work with just pass (?)

        rospy.loginfo("state_aggregator initializing -- dvlref set - pos:" + str(self.dvl.pos_dvlref_global) + ", q: " + str(self.dvl.q_dvlref_global))
        rospy.loginfo("state_aggregator initializing -- dvl active")

    def update_state(self, _):
        pos_world_global = self.pos_auv_world()
        pos_auv_world = self.pos_auv_world()
        q_auv_world = self.q_auv_world()
        euler_auv_world = self.euler_auv_world()
        w_auv = self.w_auv()

        # publish AUV pose (relative to world)
        position = Point(x=pos_auv_world[0], y=pos_auv_world[1], z=pos_auv_world[2]) 
        orientation = Quaternion(x=q_auv_world.x, y=q_auv_world.y, z=q_auv_world.z, w=q_auv_world.w)
        pose = Pose(position, orientation)
        self.pub_auv.publish(pose)

        # publish individual degrees of freedom
        self.pub_x.publish(pos_auv_world[0])
        self.pub_y.publish(pos_auv_world[1])
        self.pub_z.publish(pos_auv_world[2])

        # publish world pose (relative to global)
        position = Point(x=self.pos_world_global[0], y=self.pos_world_global[1], z=self.pos_world_global[2]) 
        orientation = Quaternion(x=self.q_world_global.x, y=self.q_world_global.y, z=self.q_world_global.z, w=self.q_world_global.w)
        pose = Pose(position, orientation)
        self.pub_world.publish(pose)

        # backwards compatibility
        self.pub_theta_x.publish(euler_auv_world[0])
        self.pub_theta_y.publish(euler_auv_world[1])
        self.pub_theta_z.publish(euler_auv_world[2])

        # angular velocity - quaternion PID
        augular_velocity = Vector3(w_auv[0], w_auv[1], w_auv[2])
        self.pub_w_auv.publish(augular_velocity)


if __name__ == '__main__':
    rospy.init_node('state_aggregator')
    sa = State_Aggregator()
    sa.initialize()
    timer = rospy.Timer(rospy.Duration(0.1), sa.update_state)
    rospy.on_shutdown(timer.shutdown)
    rospy.spin()
