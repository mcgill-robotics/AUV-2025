#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

class StateTheta:
    def __init__(self):
        rospy.Subscriber('state_theta_x', Float64, self.updateThetaX)
        rospy.Subscriber('state_theta_y', Float64, self.updateThetaY)
        rospy.Subscriber('state_theta_z', Float64, self.updateThetaZ)
        self.x = None
        self.y = None
        self.z = None
    def updateThetaX(self, msg):
        self.x = float(msg.data)
    def updateThetaY(self, msg):
        self.y = float(msg.data)
    def updateThetaZ(self, msg):
        self.z = float(msg.data)
    def getThetaX(self): return self.x
    def getThetaY(self): return self.y
    def getThetaZ(self): return self.z

def republishSetpoint(msg, args):
    pub, state = args
    #get the target angle of the setpoint
    target_angle = float(msg.data)
    #get current angle of AUV from the corresponding state function
    current_angle = state()
    if current_angle == None:
        print("Cannot re-publish theta setpoint since state is still None.")
        pub.publish(Float64(target_angle))
        return

    #increase/decrease the setpoint angle by 360 until it is within 180 degrees of the AUV's current angle
    while abs(target_angle-current_angle) > 180:
        if target_angle > current_angle: target_angle -= 360
        else: target_angle += 360
    pub.publish(Float64(target_angle))

#NODE FUNCTION: republish incoming theta setpoints for the AUV so that they are as close to the current theta state of the AUV as possible
#basically modulo the setpoint until it is within 180 degrees of where the AUV is currently facing
#this way we avoid the AUV doing more than 180 degree movements to reach a setpoint, instead it will rotate via the shortest path
#since the AUV state theta is continuous i.e. does not stop at 0/360, it keeps going into the negatives / above 360

if __name__ == '__main__':
    rospy.init_node('theta_setpoint_republish')
    state_theta = StateTheta()
    x_pub = rospy.Publisher('theta_x_setpoint_adjusted', Float64, queue_size=1)
    y_pub = rospy.Publisher('theta_y_setpoint_adjusted', Float64, queue_size=1)
    z_pub = rospy.Publisher('theta_z_setpoint_adjusted', Float64, queue_size=1)
    #pass the publisher and correct StateTheta callable get() function to the callback
    rospy.Subscriber('theta_x_setpoint', Float64, republishSetpoint, (x_pub, state_theta.getThetaX))
    rospy.Subscriber('theta_y_setpoint', Float64, republishSetpoint, (y_pub, state_theta.getThetaY))
    rospy.Subscriber('theta_z_setpoint', Float64, republishSetpoint, (z_pub, state_theta.getThetaZ))
    rospy.spin()