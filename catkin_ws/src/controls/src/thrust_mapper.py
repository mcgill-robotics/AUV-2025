#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Wrench
from auv_msgs.msg import ThrusterCommands

'''
- superimposer will publish message (type Wrench) onto topic /controls/wrench
- thrust_mapper will read message from /controls/wrench and convert the Wrench into PWM commands
- thrust_mapper will publish the pwm commands onto /servo_pos topic
'''

''' Transformation Matrices '''

# TODO - values need to be re-derived
pwm_to_wrench = matrix([
        # Fx Fy Fz Mx My Mz
        [1, 0, 0, 0, 0, 0],             # surge_port
        [1, 0, 0, 0, 0, 0],             # surge_starboard
        [0, 1, 0, 0, 0, -0.720],        # sway_bow
        [0, 1, 0, 0, 0, 0.340],         # sway_stern
        [0, 0, 1, -0.162, -0.240, 0],   # heave_bow_port
        [0, 0, 1, 0.162, -0.240, 0],    # heave_bow_starboard
        [0, 0, 1, -0.162, 0.460, 0],    # heave_stern_port
        [0, 0, 1, 0.162, 0.460, 0]      # heave_stern_starboard
    ]).T

# TODO - likely the components of the thrust produced at each motor will be determined, 
# the position of the servo is determined trigonometrically
wrench_to_servos = np.linalg.pinv(pwm_to_wrench) # TODO - 4x8 matrix defining position of servos
wrench_to_thrusters = np.linalg.pinv(pwm_to_wrench) # TODO - 4x8 matrix defining action of thrusters


'''
Callback when something is published to /controls/wrench topic by superimposer
maps the wrench onto PWM voltages and publishes them to /servo_pos

@param msg - the published message of type Wrench
'''
def wrench_callback(data):

    wrench = matrix([data.force.x, data.force.y, data.force.z, 
                    data.torque.x, data.torque.y, data.torque.z]).T
    
    servos = np.matmul(wrench_to_servos, wrench)
    thrusters = np.matmul(wrench_to_thrusters, wrench)




'''
main point of execution
'''
def thrust_mapper():
    rospy.init_node('thrust_mapper')

    rospy.Subscriber('/controls/wrench', Wrench, wrench_callback)
    pub = rospy.Publisher('/servo_pos', ThrusterCommands)

    rospy.spin()



if __name__ == '__main__':
       try:
           thrust_mapper()
        except rospy.ROSInterruptException:
           pass