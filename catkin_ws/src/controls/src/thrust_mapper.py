#!/usr/bin/env python3

import rospy
import numpy as np
import scipy as sp
from geometry_msgs.msg import Wrench
from auv_msgs.msg import ThrusterCommands
from dimensions import D


current_state = 0, 0, 0, 0, 0, 0, 0, 0 # order is t1, t2, t3, t4, s1, s2, s3, s4

'''
- superimposer will publish message (type Wrench) onto topic /controls/wrench
- thrust_mapper will read message from /controls/wrench and convert the Wrench into PWM commands
- thrust_mapper will publish the pwm commands onto /servo_pos topic
'''


def create_equations(wrench):
    fx, fy, fz, mx, my, mz = wrench

    def equations(p):
        t1, t2, t3, t4, s1, s2, s3, s4 = p
        return (
                -t2*np.cos(s2) + t4*np.cos(s4) - fx,
                t1*np.cos(s1) - t3*np.cos(s3) - fy,
                t1*np.sin(s1) + t2*np.sin(s2) + t3*np.sin(s3) + t4*np.sin(s4) - fz,
                D*t2*np.sin(s2) - D*t4*np.sin(s4) - mx,
                -D*t1*np.sin(s1) + D*t3*np.sin(s3) - my,
                 D*t1*np.cos(s1) + D*t2*np.cos(s2) + D*t3*np.cos(s3) + D*t4*np.cos(s4) - mz
                 )
    return equations

def solve_state(wrench):

    return fsolve(create_equations(wrench), current_state) # t1, t2, t3, t4, s1, s2, s3, s4


def force_to_pwm(forces):
    return forces # TODO - map forces to pwm array


def angles_to_pwm(angles):
    return angles #TODO - map angles to pwm array

'''
# TODO - likely the components of the thrust produced at each motor will be determined,
Callback when something is published to /controls/wrench topic by superimposer
maps the wrench onto PWM voltages and publishes them to /servo_pos

@param msg - the published message of type Wrench
'''

def wrench_callback(data):

    wrench = (data.force.x, data.force.y, data.force.z,
                    data.torque.x, data.torque.y, data.torque.z)
    next_state = solve_state(wrench)
    thrusters = force_to_pwm(next_state[0:4])
    servos = angles_to_pwm(next_state[4:])
    pub.publish(thrusters, servos)
    current_state = next_state

'''
main point of execution
'''
def thrust_mapper():
    rospy.init_node('thrust_mapper')

    rospy.Subscriber('/controls/wrench', Wrench, wrench_callback)
    pub = rospy.Publisher('/servo_pos', ThrusterCommands, queue_size=10)

    rospy.spin()



if __name__ == '__main__':
    try:
        thrust_mapper()
    except rospy.ROSInterruptException:
        pass
