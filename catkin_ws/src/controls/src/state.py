import rospy

from math import acos, asin
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_multiply

class AUV_State:
    def __init__(self, pose_msg):
        self.position = Position.from_msg(pose_msg.position)
        self.orientation = Orientation.from_msg(pose_msg.orientation)

    def as_pose(self):
        pos = Point(x=self.position.x, 
                    y=self.position.y, 
                    z=self.position.z)
        orient = Quaternion(x=self.orientation.x, 
                    y=self.orientation.y,
                    z=self.orientation.z,
                    w=self.orientation.w)
        return Pose(pos, orient)

'''
To be used for setting PID setpoints
'''
class Setpoint_State(AUV_State):
    def __init__(self, pose_msg):
        super().__init__(pose_msg)

        self.x_pub = rospy.Publisher("x_setpoint", Float64, queue_size=50)
        self.y_pub = rospy.Publisher("y_setpoint", Float64, queue_size=50)
        self.z_pub = rospy.Publisher("z_setpoint", Float64, queue_size=50)
        self.theta_pub = rospy.Publisher("theta_setpoint", Float64, queue_size=50)
        self.dtheta_pub = rospy.Publisher("dtheta_setpoint", Float64, queue_size=50)

    def update(self, pose_msg):
        self.position.update(pose_msg.position)
        self.orientation.update(pose_msg.orientation)


    def publish(self):
        self.x_pub.publish(Float64(self.position.x))
        self.y_pub.publish(Float64(self.position.y))
        self.z_pub.publish(Float64(self.position.z))
        self.theta_pub.publish(Float64(self.orientation.theta))


    def publish_angle_diff(self, curr_state):
        # dtheta is angle between this and the current state
        q3 = curr_state.orientation.transform_to(self.orientation)
        self.dtheta_pub.publish(Float64(q3.theta))



'''
To be used for setting PID state
'''
class Perceived_State(AUV_State):
    def __init__(self, pose_msg):
        super().__init__(pose_msg)

        self.x_pub = rospy.Publisher("x_state", Float64, queue_size=50)
        self.y_pub = rospy.Publisher("y_state", Float64, queue_size=50)
        self.z_pub = rospy.Publisher("z_state", Float64, queue_size=50)
        self.theta_pub = rospy.Publisher("theta_state", Float64, queue_size=50)


    def publish(self):
        self.x_pub.publish(Float64(self.position.x))
        self.y_pub.publish(Float64(self.position.y))
        self.z_pub.publish(Float64(self.position.z))
        self.theta_pub.publish(Float64(self.orientation.theta))


'''
encapsulates a point describing robot's
location relative to a datum (global) axis
'''
class Position:
    
    def from_msg(pos_msg):
        x = pos_msg.x
        y = pos_msg.y
        z = pos_msg.z
        return Position(x, y, z)


    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


'''
encapsulates a quaternion describing robot's
rotation relative to a datum (global) axis
'''
class Orientation:
    
    def from_msg(orient_msg):
        x = orient_msg.x
        y = orient_msg.y
        z = orient_msg.z
        w = orient_msg.w
        return Orientation(x, y, z, w)

    def __init__(self, x, y, z, w):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
        self.theta = acos(self.w)*2


    def inverse(self):
        return Orientation(-self.x, -self.y, -self.z, self.w)


    '''
    quaternion multiplication q1*q2
    '''
    def mult(self, q2):
        q1_list = [self.x, self.y, self.z, self.w]
        q2_list = [q2.x, q2.y, q2.z, q2.w]
        q3 = quaternion_multiply(q1_list, q2_list)
        return Orientation(*q3)

    '''
    returns an Orientation that
    represents a purely rotational transformation
    necessary to bring the robot from 
    this orientation to orientation q2
    '''
    def transform_to(self, q2):
        return q2.mult(self.inverse())

    def axis_of_rotation(self):
        a = asin(self.theta)*2
        return [a*self.x, a*self.y, a*self.z]
