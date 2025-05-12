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
import threading
import keyboard

# -------------------- Thruster Indices --------------------
BACK_LEFT = 0
HEAVE_BACK_LEFT = 1
HEAVE_FRONT_LEFT = 2
FRONT_LEFT = 3
FRONT_RIGHT = 4
HEAVE_FRONT_RIGHT = 5
HEAVE_BACK_RIGHT = 6
BACK_RIGHT = 7

# -------------------- Global Safety Flag --------------------
emergency_triggered = False

def emergency_reset_loop():
    global emergency_triggered
    while not rospy.is_shutdown():
        if keyboard.is_pressed('r'):
            rospy.logwarn("[SAFETY] Reset key pressed. Resetting thrusters and restarting test.")
            pwm_pub.publish(ThrusterMicroseconds(microseconds=[1500]*8))
            emergency_triggered = True
            rospy.sleep(0.5)

# -------------------- Safe Sleep Function --------------------
def safe_sleep(duration):
    global emergency_triggered
    interval = 0.1  # seconds
    elapsed = 0.0
    while elapsed < duration and not emergency_triggered:
        rospy.sleep(interval)
        elapsed += interval

# -------------------- Force Settings --------------------
frwd_force = None
vert_force = None
g = 9.81

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
rospy.sleep(2.0)

# Start emergency safety thread
safety_thread = threading.Thread(target=emergency_reset_loop, daemon=True)
safety_thread.start()

reset_cmd = ThrusterMicroseconds(microseconds=[1500]*8)
use_old_coeffs = True

# -------------------- Main Test Loop --------------------
while not rospy.is_shutdown():
    emergency_triggered = False

    user_input = input("\n[INPUT] Press ENTER to repeat test, or type 'q' to quit: ")
    if user_input.lower() == 'q':
        print("[INFO] Exiting test loop.")
        break

    go = False
    while not go:
        user_input = input("\n[INPUT] Type 'n' for new coeffs, or 'o' for old coeffs: ")    
        if user_input.lower() == "n":
            use_old_coeffs = False
            go = True
        elif user_input.lower() == "o":
            use_old_coeffs = True
            go = True

    go = False

    if use_old_coeffs:
        user_input = input("\n[INPUT] Specify FORWARD force per thruster, in kg: ")  
        frwd_force = float(user_input)

        while not go:
            pwm_pub.publish(reset_cmd) # For safety
            user_input = input("\n[INPUT] Specify VERTICAL force per thruster, in kg: ") 
            vert_force = float(user_input)
            print(f"[INFO] Using OLD mapping at {vert_force:.2f} kg vertical and {frwd_force:.2f} kg forward")

            vert_pwm = force_to_pwm(vert_force, old_coeffs[0], old_coeffs[1])
            front_pwm = force_to_pwm(-frwd_force, old_coeffs[0], old_coeffs[1])
            back_pwm = force_to_pwm(frwd_force, old_coeffs[0], old_coeffs[1])

            hover_pwm = [1500] * 8
            for i in [HEAVE_BACK_LEFT, HEAVE_FRONT_LEFT, HEAVE_FRONT_RIGHT, HEAVE_BACK_RIGHT]:
                hover_pwm[i] = vert_pwm

            print(f"[INFO] Publishing hover command: {hover_pwm}")
            pwm_pub.publish(ThrusterMicroseconds(microseconds=hover_pwm))
            safe_sleep(15.0)

            if emergency_triggered:
                continue

            user_input = input("\n[INPUT] Type 'g' to continue or ENTER to repeat: ") 
            if user_input.lower() == "g":
                go = True

        forward_pwm = hover_pwm.copy()
        forward_pwm[FRONT_LEFT] = front_pwm
        forward_pwm[FRONT_RIGHT] = front_pwm
        forward_pwm[BACK_LEFT] = back_pwm
        forward_pwm[BACK_RIGHT] = back_pwm

        print(f"[INFO] Publishing forward command: {forward_pwm}")
        pwm_pub.publish(ThrusterMicroseconds(microseconds=forward_pwm))
        safe_sleep(5.0)

        if emergency_triggered:
            continue

        pwm_pub.publish(reset_cmd)
        print("[INFO] Test complete. Thrusters reset to neutral.")

    else: # Using New Mapping
        user_input = input("\n[INPUT] Specify FORWARD force per thruster, in kg: ")  
        frwd_force = float(user_input) * g

        while not go:
            user_input = input("\n[INPUT] Specify VERTICAL force per thruster, in kg: ") 
            vert_force = float(user_input) * g
            print(f"[INFO] Using NEW mapping at {vert_force:.2f} N vertical and {frwd_force:.2f} N forward")

            hover_pwm = [1500, force_to_pwm_thruster(7, vert_force), force_to_pwm_thruster(6, vert_force), 1500,
                         1500, force_to_pwm_thruster(3, vert_force), force_to_pwm_thruster(2, vert_force), 1500]

            print(f"[INFO] Publishing hover command: {hover_pwm}")
            pwm_pub.publish(ThrusterMicroseconds(microseconds=hover_pwm))
            safe_sleep(15.0)

            if emergency_triggered:
                continue

            user_input = input("\n[INPUT] Type 'g' to continue or ENTER to repeat: ") 
            if user_input.lower() == "g":
                go = True

        forward_pwm = hover_pwm.copy()
        forward_pwm[BACK_LEFT] = force_to_pwm_thruster(8, frwd_force)
        forward_pwm[FRONT_LEFT] = force_to_pwm_thruster(5, -frwd_force)
        forward_pwm[FRONT_RIGHT] = force_to_pwm_thruster(4, -frwd_force)
        forward_pwm[BACK_RIGHT] = force_to_pwm_thruster(1, frwd_force)

        print(f"[INFO] Publishing forward command: {forward_pwm}")
        pwm_pub.publish(ThrusterMicroseconds(microseconds=forward_pwm))
        safe_sleep(5.0)

        if emergency_triggered:
            continue

        pwm_pub.publish(reset_cmd)
        print("[INFO] Test complete. Thrusters reset to neutral.")