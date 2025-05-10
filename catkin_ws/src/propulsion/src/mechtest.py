#!/usr/bin/env python3

"""
Script to compare old and new force-to-PWM mappings.

Procedure:
1. Use the old mapping to dive and maintain constant depth.
2. Move forward for 5 seconds using front and rear thrusters.
3. Measure drift (e.g., with tape or visual markers).
4. Repeat test using the new mapping to compare drift performance.
"""

import rospy
from auv_msgs.msg import ThrusterMicroseconds
from thrust_mapper_utils import force_to_pwm, force_to_pwm_thruster

# -------------------- Thruster Indices for Readability --------------------
# The indexing below is the one used by the software team. Mech uses a different one (in reverse), 
# disregard any confusion when we call force_to_pwm_thruster functions

BACK_LEFT = 0 # 8 for mech
HEAVE_BACK_LEFT = 1 # 7 for mech
HEAVE_FRONT_LEFT = 2 # 6 for mech 
FRONT_LEFT = 3 # 5 for mech
FRONT_RIGHT = 4 # 4 for mech
HEAVE_FRONT_RIGHT = 5 # 3 for mech
HEAVE_BACK_RIGHT = 6 # 2 for mech
BACK_RIGHT = 7 # 1 for mech

# -------------------- Force Settings --------------------
frwd_force = 0.5 * 9.81   # N
vert_force = 0.375 * 9.81  # N

# -------------------- Old Mapping Coefficients --------------------
old_coeffs = (
    [1.3116834437073399, 1.6655229499580056e1, 8.4596303821198617e1, 2.1943237103469241e2,
     3.1133962497995367e2, 2.5705773429064880e2, 2.3999978362959104e2, 1.4701043632380542e3],
    [2.1398707591286295e-1, -3.5559291204780612, 2.3661746039755371e1, -8.0478019175136623e1,
     1.4980771349508325e2, -1.6227874418158476e2, 1.9317247519327023e2, 1.5299083405100268e3]
)

# -------------------- ROS Setup --------------------
rospy.init_node("mech_test")
pwm_pub = rospy.Publisher("/propulsion/microseconds", ThrusterMicroseconds, queue_size=1)
rospy.sleep(2.0)  # Allow time for publishers to initialize

# -------------------- Reset Command --------------------
reset = [1500] * 8
reset_cmd = ThrusterMicroseconds(microseconds=reset)

# -------------------- Config --------------------
use_old_coeffs = True  # Set to False to test the new mapping

# -------------------- PWM Command Generation --------------------
if use_old_coeffs == True:
    print(f"[INFO] Using OLD mapping at {frwd_force:.2f} N forward force per thruster")

    # Compute PWM values from force using old mapping
    vert_pwm = force_to_pwm(vert_force, old_coeffs[0], old_coeffs[1])
    front_pwm = force_to_pwm(-frwd_force, old_coeffs[0], old_coeffs[1])
    back_pwm = force_to_pwm(frwd_force, old_coeffs[0], old_coeffs[1])

    # Hover (constant depth)
    hover_pwm = [1500] * 8
    for i in [HEAVE_BACK_LEFT, HEAVE_FRONT_LEFT, HEAVE_FRONT_RIGHT, HEAVE_BACK_RIGHT]:
        hover_pwm[i] = vert_pwm

    print(f"[INFO] Publishing hover command: {hover_pwm}")
    pwm_pub.publish(ThrusterMicroseconds(microseconds=hover_pwm))
    rospy.sleep(15.0)

    # Forward motion
    forward_pwm = hover_pwm.copy()
    forward_pwm[FRONT_LEFT] = front_pwm
    forward_pwm[FRONT_RIGHT] = front_pwm
    forward_pwm[BACK_LEFT] = back_pwm
    forward_pwm[BACK_RIGHT] = back_pwm
    
    print(f"[INFO] Publishing forward command: {forward_pwm}")
    pwm_pub.publish(ThrusterMicroseconds(microseconds=forward_pwm))
    rospy.sleep(5.0)

    # Reset all thrusters to idle
    pwm_pub.publish(reset_cmd)
    print("[INFO] Test complete. Thrusters reset to neutral.")

else: # We now use new thruster mapping
    # Hover (constant depth)
    hover_pwm = [1500, force_to_pwm_thruster(7,vert_force), force_to_pwm_thruster(6, vert_force), 1500,
                 1500, force_to_pwm_thruster(3, vert_force), force_to_pwm_thruster(2, vert_force), 1500 ]
    # the thruster mapping uses the mech indices. 
    
    print(f"[INFO] Publishing hover command: {hover_pwm}")
    pwm_pub.publish(ThrusterMicroseconds(microseconds=hover_pwm))
    rospy.sleep(15.0)

    # Forward Motion
    forward_pwm = hover_pwm.copy()
    forward_pwm[BACK_LEFT] = force_to_pwm_thruster(8,frwd_force)
    forward_pwm[FRONT_LEFT] = force_to_pwm_thruster(5,-1.0 * frwd_force)
    forward_pwm[FRONT_RIGHT] = force_to_pwm_thruster(4,-1.0 * frwd_force)
    forward_pwm[BACK_RIGHT] = force_to_pwm_thruster(1,frwd_force)

    print(f"[INFO] Publishing forward command: {forward_pwm}")
    pwm_pub.publish(ThrusterMicroseconds(microseconds=forward_pwm))
    rospy.sleep(5.0)

    # Reset all thrusters to idle
    pwm_pub.publish(reset_cmd)
    print("[INFO] Test complete. Thrusters reset to neutral.")
