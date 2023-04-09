import rospy
from sensor_msgs.msg import Float64

def updateX(msg):
    global state_x
    state_x = float(msg.data)
def updateY(msg):
    global state_y
    state_y = float(msg.data)
def updateZ(msg):
    global state_z
    state_z = float(msg.data)
def updateThetaX(msg):
    global state_theta_x
    state_theta_x = float(msg.data)
def updateThetaY(msg):
    global state_theta_y
    state_theta_y = float(msg.data)
def updateThetaZ(msg):
    global state_theta_z
    state_theta_z = float(msg.data)

def getPosition(): return (state_x, state_y, state_z)

def getRotation(): return (state_theta_x, state_theta_y, state_theta_z)

x_pos_sub = rospy.Subscriber('state_x', Float64, updateX)
y_pos_sub = rospy.Subscriber('state_y', Float64, updateY)
z_pos_sub = rospy.Subscriber('state_z', Float64, updateZ)
theta_x_sub = rospy.Subscriber('state_theta_x', Float64, updateThetaX)
theta_y_sub = rospy.Subscriber('state_theta_y', Float64, updateThetaY)
theta_z_sub = rospy.Subscriber('state_theta_z', Float64, updateThetaZ)