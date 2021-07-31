#!/usr/bin/env python

import rospy
from rospkg import RosPack
from geometry_msgs.msg import Wrench
from auv_msgs.msg import ThrusterCommands
from numpy import matrix, interp, genfromtxt
from numpy.linalg import pinv


def wrench_to_thrust(thrust_char):
    '''
    wrench_to_thrust returns a function, configured with thrust
    characterisations, that maps a Wrench into thrust voltages
    '''
    thrusters = [
        ThrusterCommands.SURGE_PORT,
        ThrusterCommands.SURGE_STARBOARD,
        ThrusterCommands.SWAY_BOW,
        ThrusterCommands.SWAY_STERN,

        ThrusterCommands.HEAVE_BOW_PORT,
        ThrusterCommands.HEAVE_BOW_STARBOARD,
        ThrusterCommands.HEAVE_STERN_PORT,
        ThrusterCommands.HEAVE_STERN_STARBOARD
    ]

    thrusterToForce = matrix([
        # Fx Fy Fz Mx My Mz
        [1, 0, 0, 0, 0, 0],      # surge_port
        [1, 0, 0, 0, 0, 0],     # surge_starboard
        [0, 1, 0, 0, 0, -0.720],       # sway_bow
        [0, 1, 0, 0, 0, 0.340],        # sway_stern

        [0, 0, 1, -0.162, -0.240, 0],   # heave_bow_port
        [0, 0, 1, 0.162, -0.240, 0],    # heave_bow_starboard
        [0, 0, 1, -0.162, 0.460, 0],   # heave_stern_port
        [0, 0, 1, 0.162, 0.460, 0]     # heave_stern_starboard
    ]).T
    forceToThruster = pinv(thrusterToForce)

    def helper(data):
        Fxyz_Mxyz = matrix(
            [data.force.x, data.force.y, data.force.z,
             data.torque.x, data.torque.y, data.torque.z]).T

        thrusterCmds = pwm(forceToThruster * Fxyz_Mxyz, thrust_char).T

        msg = [0] * len(thrusters)
        for t, x in zip(thrusters, thrusterCmds.tolist()[0]):
            msg[t] = x
        thrust_pub.publish(ThrusterCommands(msg))

    return helper


def pwm(thrust, t100):
    min_thrust = 0.01
    pwm = interp(thrust, t100[:, 0], t100[:, 1])
    pwm[abs(thrust) < min_thrust] = 0
    return pwm

if __name__ == '__main__':
    rospy.init_node('thrust_mapper')

    # Loads the characterizations for the thrusters
    char_file = RosPack().get_path('controls') + \
        '/config/t100_characterization.csv'

    t100_pwm = genfromtxt(char_file, delimiter=',')
    t100_pwm = t100_pwm[t100_pwm[:, 1].argsort()]

    thrust_pub = rospy.Publisher('thrust_cmds', ThrusterCommands, queue_size=5)
    sub = rospy.Subscriber('/controls/wrench',
                           Wrench,
                           wrench_to_thrust(t100_pwm))
    rospy.spin()
