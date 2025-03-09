#!/usr/bin/env python3

import rospy
from auv_msgs.msg import ThrusterMicroseconds
from thrust_mapper_utils import *
from geometry_msgs.msg import Wrench

force_amt = 0.1  # 10%

# TODO: update if necessary
reset = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]

reset_cmd = ThrusterMicroseconds(reset)
pub = rospy.Publisher("/propulsion/microseconds", ThrusterMicroseconds, queue_size=1)
effort_pub = rospy.Publisher("/controls/effort", Wrench, queue_size=1)
rospy.sleep(1)

rospy.init_node("thrusters_test")


def simultaneous_forwards_test():
    while not rospy.is_shutdown():
        print("- spinning at " + str(100 * force_amt) + "% max forwards force for 1s")
        
        cmd = [force_to_pwm(force_amt * MAX_FWD_FORCE)] * 8
        pub.publish(cmd)
        rospy.sleep(1.0)
        pub.publish(reset_cmd)

        print("1. repeat test")
        print("2. proceed")
        choice = input()
        if choice != "1":
            break

# just type the thruster you want and it will run
def optimized_dry_test(t):
    print("- spinning at " + str(100 * force_amt) + "% max forwards force for 1s")
    cmd = reset.copy()
    cmd[t - 1] = force_to_pwm(force_amt * MAX_FWD_FORCE * thruster_mount_dirs[t-1])
    pub.publish(cmd)
    rospy.sleep(1.0)
    pub.publish(reset_cmd)

def reset_thrusters():
    pub.publish(reset_cmd)
    print("Safely shutting down thrusters")


def re_arm():
    rospy.sleep(1)
    msg1 = ThrusterMicroseconds([1500] * 8)
    msg2 = ThrusterMicroseconds([1540] * 8)

    pub.publish(msg1)
    rospy.sleep(0.5)
    pub.publish(msg2)
    rospy.sleep(0.5)
    pub.publish(msg1)


rospy.on_shutdown(reset_thrusters)
while not rospy.is_shutdown():
    print("========== Thrusters Test ==========")
    print(
        "refer to images in dry-test/images for labelled diagrams of thrusters on the AUV"
    )
    print("1. select thruster to test")
    print("2. test thrusters simultaneously")
    print("3. re-arm")
    print("4. Keyboard control")
    print("5. exit")
    choice = input()

    if choice == "1":
        while True:
            choice = int(input("select thruster to test (1-8): "))
            if 1 <= choice and choice <= 8:
                optimized_dry_test(choice)
            else:
                break
    elif choice == "2":
        simultaneous_forwards_test()
    elif choice == "3":
        re_arm()
    elif choice == "4":
        print("NOTE: Uses thrust mapper")
        print(" > WASD for SURGE/SWAY")
        print(" > Q/E for UP/DOWN")
        print(" > IJKL for PITCH/YAW")
        print(" > U/O for ROLL")
        print(" > ESC to exit")
        while True:
            desired_effort = Wrench()
            desired_effort.force.x = 0
            desired_effort.force.y = 0
            desired_effort.force.z = 0
            desired_effort.torque.x = 0
            desired_effort.torque.y = 0
            desired_effort.torque.z = 0

            user_input = input("Enter a command (WASD/QE/IJKL/UO/ESC to exit): ").lower()

            if user_input == "esc":
                break

                # Handle the user input and adjust forces and torques accordingly
            if user_input == "w":
                desired_effort.force.x += force_amt * MAX_FWD_FORCE
            elif user_input == "s":
                desired_effort.force.x += force_amt * MAX_BKWD_FORCE
            elif user_input == "d":
                desired_effort.force.y += force_amt * MAX_FWD_FORCE
            elif user_input == "a":
                desired_effort.force.y += force_amt * MAX_BKWD_FORCE
            elif user_input == "e":
                desired_effort.force.z += force_amt * MAX_FWD_FORCE
            elif user_input == "q":
                desired_effort.force.z += force_amt * MAX_BKWD_FORCE
            elif user_input == "o":
                desired_effort.torque.x += force_amt * MAX_FWD_FORCE
            elif user_input == "u":
                desired_effort.torque.x += force_amt * MAX_BKWD_FORCE
            elif user_input == "i":
                desired_effort.torque.y += force_amt * MAX_FWD_FORCE
            elif user_input == "k":
                desired_effort.torque.y += force_amt * MAX_BKWD_FORCE
            elif user_input == "l":
                desired_effort.torque.z += force_amt * MAX_FWD_FORCE
            elif user_input == "j":
                desired_effort.torque.z += force_amt * MAX_BKWD_FORCE

            effort_pub.publish(desired_effort)
    else:
        break
