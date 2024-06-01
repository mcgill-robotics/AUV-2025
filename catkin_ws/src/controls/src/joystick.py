#!/usr/bin/env python3

import rospy
from auv_msgs.msg import ThrusterMicroseconds
from geometry_msgs.msg import Wrench
import keyboard

low_force_amt = 0.0025  # 0.25%
force_amt = 1.0

# forces produced by T200 thruster at 14V (N)
MAX_FWD_FORCE = 4.52 * 9.81
MAX_BKWD_FORCE = -3.52 * 9.81

reset_cmd = ThrusterMicroseconds([1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500])
rospy.init_node("joystick")
rospy.sleep(7)
effort_pub = rospy.Publisher("/controls/effort", Wrench, queue_size=1)


pub = rospy.Publisher("/propulsion/microseconds", ThrusterMicroseconds, queue_size=1)


def reset_thrusters():
    pub.publish(reset_cmd)
    print("Safely shutting down thrusters")


rospy.on_shutdown(reset_thrusters)
while not rospy.is_shutdown():
    print(" > WASD for SURGE/SWAY")
    print(" > Q/E for UP/DOWN")
    print(" > IJKL for PITCH/YAW")
    print(" > U/O for ROLL")
    print(" > hold SPACE for max. force")
    while True:
        desired_effort = Wrench()
        desired_effort.force.x = 0
        desired_effort.force.y = 0
        desired_effort.force.z = 0
        desired_effort.torque.x = 0
        desired_effort.torque.y = 0
        desired_effort.torque.z = 0

        current_force_amt = force_amt if keyboard.is_pressed("space") else low_force_amt

        if keyboard.is_pressed("esc"):
            break
        if keyboard.is_pressed("w"):
            desired_effort.force.x = (
                desired_effort.force.x + current_force_amt * MAX_FWD_FORCE
            )
        if keyboard.is_pressed("s"):
            desired_effort.force.x = (
                desired_effort.force.x - current_force_amt * MAX_BKWD_FORCE
            )
        if keyboard.is_pressed("a"):
            desired_effort.force.y = (
                desired_effort.force.y + current_force_amt * MAX_FWD_FORCE
            )
        if keyboard.is_pressed("d"):
            desired_effort.force.y = (
                desired_effort.force.y - current_force_amt * MAX_BKWD_FORCE
            )
        if keyboard.is_pressed("q"):
            desired_effort.force.z = (
                desired_effort.force.z + current_force_amt * MAX_FWD_FORCE
            )
        if keyboard.is_pressed("e"):
            desired_effort.force.z = (
                desired_effort.force.z - current_force_amt * MAX_BKWD_FORCE
            )
        if keyboard.is_pressed("o"):
            desired_effort.torque.y = (
                desired_effort.torque.x + current_force_amt * MAX_FWD_FORCE
            )
        if keyboard.is_pressed("u"):
            desired_effort.torque.y = (
                desired_effort.torque.x - current_force_amt * MAX_BKWD_FORCE
            )
        if keyboard.is_pressed("i"):
            desired_effort.torque.y = (
                desired_effort.torque.y + current_force_amt * MAX_FWD_FORCE
            )
        if keyboard.is_pressed("k"):
            desired_effort.torque.y = (
                desired_effort.torque.y - current_force_amt * MAX_BKWD_FORCE
            )
        if keyboard.is_pressed("j"):
            desired_effort.torque.z = (
                desired_effort.torque.z + current_force_amt * MAX_FWD_FORCE
            )
        if keyboard.is_pressed("l"):
            desired_effort.torque.z = (
                desired_effort.torque.z - current_force_amt * MAX_BKWD_FORCE
            )

        effort_pub.publish(desired_effort)
