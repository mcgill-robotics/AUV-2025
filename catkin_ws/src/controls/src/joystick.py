#!/usr/bin/env python3

import rospy
from auv_msgs.msg import ThrusterMicroseconds
from std_msgs.msg import Float64
import keyboard

# forces produced by T200 thruster at 14V (N)
MAX_FWD_FORCE = 4.52 * 9.81
MAX_BKWD_FORCE = -3.52 * 9.81

reset_cmd = ThrusterMicroseconds([1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500])
rospy.init_node("joystick")
rospy.sleep(7)
x_pub = rospy.Publisher("/controls/force/global/x", Float64, queue_size=1)
y_pub = rospy.Publisher("/controls/force/global/y", Float64, queue_size=1)
z_pub = rospy.Publisher("/controls/force/global/z", Float64, queue_size=1)
roll_pub = rospy.Publisher("/controls/torque/roll", Float64, queue_size=1)
pitch_pub = rospy.Publisher("/controls/torque/pitch", Float64, queue_size=1)
yaw_pub = rospy.Publisher("/controls/torque/yaw", Float64, queue_size=1)


pub = rospy.Publisher("/propulsion/microseconds", ThrusterMicroseconds, queue_size=1)


def reset_thrusters():
    pub.publish(reset_cmd)
    print("Safely shutting down thrusters")


rospy.on_shutdown(reset_thrusters)
while not rospy.is_shutdown():
    print("NOTE: Launch controls.launch and propulsion.launch to use joystick.")
    print(" > WASD for SURGE/SWAY")
    print(" > Q/E for UP/DOWN")
    print(" > IJKL for PITCH/YAW")
    print(" > U/O for ROLL")
    print(" > hold SPACE for max. force")
    while True:
        desired_x_force = 0
        desired_y_force = 0
        desired_z_force = 0
        desired_x_torque = 0
        desired_y_torque = 0
        desired_z_torque = 0

        current_force_amt = (
            float(rospy.get_param("joystick_max_force"))
            if keyboard.is_pressed("space")
            else float(rospy.get_param("joystick_dry_test_force"))
        )

        if keyboard.is_pressed("esc"):
            break
        if keyboard.is_pressed("w"):
            desired_x_force += current_force_amt * MAX_FWD_FORCE
        if keyboard.is_pressed("s"):
            desired_x_force += current_force_amt * MAX_BKWD_FORCE
        if keyboard.is_pressed("a"):
            desired_y_force += current_force_amt * MAX_FWD_FORCE
        if keyboard.is_pressed("d"):
            desired_y_force += current_force_amt * MAX_BKWD_FORCE
        if keyboard.is_pressed("q"):
            desired_z_force += current_force_amt * MAX_FWD_FORCE
        if keyboard.is_pressed("e"):
            desired_z_force += current_force_amt * MAX_BKWD_FORCE
        if keyboard.is_pressed("o"):
            desired_y_torque += current_force_amt * MAX_FWD_FORCE
        if keyboard.is_pressed("u"):
            desired_y_torque += current_force_amt * MAX_BKWD_FORCE
        if keyboard.is_pressed("i"):
            desired_y_torque += current_force_amt * MAX_FWD_FORCE
        if keyboard.is_pressed("k"):
            desired_y_torque += current_force_amt * MAX_BKWD_FORCE
        if keyboard.is_pressed("j"):
            desired_z_torque += current_force_amt * MAX_FWD_FORCE
        if keyboard.is_pressed("l"):
            desired_z_torque += current_force_amt * MAX_BKWD_FORCE

        x_pub.publish(desired_x_force)
        y_pub.publish(desired_y_force)
        z_pub.publish(desired_z_force)
        roll_pub.publish(desired_x_torque)
        pitch_pub.publish(desired_y_torque)
        yaw_pub.publish(desired_z_torque)
