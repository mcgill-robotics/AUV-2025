#!/usr/bin/env python3

import rospy
import keyboard
from tf import transformations
import numpy as np
import quaternion
import cv2
import os
from cv_bridge import CvBridge

from joystick_utils import *

from auv_msgs.msg import ThrusterMicroseconds
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Quaternion, Pose
from sensor_msgs.msg import Image


# Note: np.quaternion -> (w, x, y, z).
#       Quaternion -> (x, y, z, w).

class Joystick:
    def __init__(self):
        self.MAX_FWD_FORCE = 4.52 * 9.81
        self.MAX_BKWD_FORCE = -3.52 * 9.81

        # Thrusters force parameters.
        self.max_force = float(rospy.get_param("joystick_max_force"))
        self.dry_test_force = float(rospy.get_param("joystick_dry_test_force"))
        self.pool_force = float(rospy.get_param("joystick_pool_force"))

        # Warning message.
        self.missing_pose_interval = float(rospy.get_param("joystick_missing_data_warning_interval"))
        self.last_warning_time = rospy.get_time()
        self.last_pose_received = rospy.get_time()
        
        # Mode names.
        self.mode_dry_test = "DRY_TEST"
        self.mode_force = "FORCE"
        self.mode_pid_global = "PID GLOBAL"
        self.mode_pid_local = "PID LOCAL"
        self.mode_current = self.mode_dry_test
        self.modes = [
            self.mode_dry_test,
            self.mode_force,
            self.mode_pid_global,
            self.mode_pid_local
        ]
        self.modes_pid = [
            self.mode_pid_global,
            self.mode_pid_local
        ]
        self.modes_execution = {
            self.mode_dry_test: self.execute_force_mode,
            self.mode_force:  self.execute_force_mode,
            self.mode_pid_local: self.execute_pid_mode,
            self.mode_pid_global: self.execute_pid_mode
        }
        self.is_esc_pressed = False      

        # Camera screenshot.
        self.folder_path = rospy.get_param("joystick_image_folder_path")
        self.down_camera_path = os.path.join(self.folder_path, "down_camera")
        self.front_camera_path = os.path.join(self.folder_path, "front_camera")
        self.file_number = 0
        self.wait_execute = False

        # Ensure the folders exist.
        os.makedirs(self.folder_path, exist_ok=True)
        os.makedirs(self.down_camera_path, exist_ok=True)
        os.makedirs(self.front_camera_path, exist_ok=True)

        self.down_camera_frame = None
        self.front_camera_frame = None

        # PID delta axis.
        self.pid_position_delta = rospy.get_param("joystick_pid_position_delta")   
        self.pid_quaternion_delta_w = rospy.get_param("joystick_pid_quaternion_delta_w")   
        self.pid_quaternion_delta_xyz = rospy.get_param("joystick_pid_quaternion_delta_xyz")   

        # Current pose.
        self.x = None
        self.y = None
        self.z = None
        self.quaternion = None

        # Subscribers.
        rospy.Subscriber("/state/pose", Pose, self.cb_pose)
        rospy.Subscriber("/vision/down_cam/image_raw", Image, self.cb_down_camera)
        rospy.Subscriber("/vision/front_cam/color/image_raw",
            Image, self.cb_front_camera
        )

        # Mode publishers.
        # [FORCE] mode.
        self.pub_x = rospy.Publisher(
            "/controls/force/global/x", Float64, queue_size=1
        )
        self.pub_y = rospy.Publisher(
            "/controls/force/global/y", Float64, queue_size=1
        )
        self.pub_z = rospy.Publisher(
            "/controls/force/global/z", Float64, queue_size=1
        )
        self.pub_roll = rospy.Publisher(
            "/controls/torque/roll", Float64, queue_size=1
        )
        self.pub_pitch = rospy.Publisher(
            "/controls/torque/pitch", Float64, queue_size=1
        )
        self.pub_yaw = rospy.Publisher(
            "/controls/torque/yaw", Float64, queue_size=1
        )
        self.pub_microseconds = rospy.Publisher(
            "/propulsion/microseconds", ThrusterMicroseconds, queue_size=1
        )
        # [PID] mode - enable.
        self.pub_pid_x_enable = rospy.Publisher(
            "/controls/pid/x/enable", Bool, queue_size=1
        )
        self.pub_pid_y_enable = rospy.Publisher(
            "/controls/pid/y/enable", Bool, queue_size=1
        )
        self.pub_pid_z_enable = rospy.Publisher(
            "/controls/pid/z/enable", Bool, queue_size=1
        )
        self.pub_pid_quat_enable = rospy.Publisher(
            "/controls/pid/quat/enable", Bool, queue_size=1
        )
        # [PID] mode - setpoints.
        self.pub_pid_x = rospy.Publisher(
            "/controls/pid/x/setpoint", Float64, queue_size=1
        )
        self.pub_pid_z = rospy.Publisher(
            "/controls/pid/z/setpoint", Float64, queue_size=1
        )
        self.pub_pid_y = rospy.Publisher(
            "/controls/pid/y/setpoint", Float64, queue_size=1
        )
        self.pub_pid_quat = rospy.Publisher(
            "/controls/pid/quat/setpoint", Quaternion, queue_size=1
        )

    def cb_pose(self, msg):
        self.x = msg.position.x
        self.y = msg.position.y
        self.z = msg.position.z
        self.quaternion = np.quaternion(
            msg.orientation.w,
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z            
        )
        self.last_pose_received = rospy.get_time()

    def cb_theta_x(self, msg):
        self.theta_x = msg.data

    def cb_theta_y(self, msg):
        self.theta_y = msg.data

    def cb_theta_z(self, msg):
        self.theta_z = msg.data

    def cb_down_camera(self, msg):
        self.down_camera_frame = msg

    def cb_front_camera(self, msg):
        self.front_camera_frame = msg

    def set_enable_pid(self, enable):
        self.pub_pid_x_enable.publish(Bool(enable))
        self.pub_pid_y_enable.publish(Bool(enable))
        self.pub_pid_z_enable.publish(Bool(enable))
        self.pub_pid_quat_enable.publish(Bool(enable))

    def mode_swap(self, mode, enable):
        rospy.sleep(1)
        if self.mode_current in self.modes_pid:
            clean_pretty_print()
        self.mode_current = mode
        print(f" > CURRENT MODE: [{self.mode_current}].")
        if self.mode_current == self.mode_dry_test:
            print(" > Hold SPACE for max force.")
        self.set_enable_pid(enable)

    def establish_key_hooks(self):
        keyboard.on_press_key(
            "esc", 
            lambda _: ( 
                setattr(self, "is_esc_pressed", True)
            )
        )
        keyboard.on_press_key(
            "0", 
            lambda _: (
                setattr(self, "wait_execute", True),
                self.mode_swap(self.mode_dry_test, False),
                setattr(self, "wait_execute", False)
            )
        )
        keyboard.on_press_key(
            "1", 
            lambda _: (
                setattr(self, "wait_execute", True),
                self.mode_swap(self.mode_force, False),
                setattr(self, "wait_execute", False)
            )
        )
        keyboard.on_press_key(
            "2", 
            lambda _: (
                setattr(self, "wait_execute", True),
                self.mode_swap(self.mode_pid_local, True),
                setattr(self, "wait_execute", False)
            )
        )
        keyboard.on_press_key(
            "3", 
            lambda _: (
                setattr(self, "wait_execute", True),
                self.mode_swap(self.mode_pid_global, True),
                setattr(self, "wait_execute", False)
            )
        )
        keyboard.on_press_key(
            "4",
            lambda _: (
                setattr(self, "wait_execute", True),
                setattr(self, 'file_number', self.file_number + 1),
                take_screenshot(
                    self.down_camera_frame, 
                    self.down_camera_path, 
                    self.file_number,
                    True if self.mode_current in self.modes_pid else False
                ),
                setattr(self, "wait_execute", False)
            )
        )
        keyboard.on_press_key(
            "5",
            lambda _: (
                setattr(self, "wait_execute", True),
                setattr(self, 'file_number', self.file_number + 1),
                take_screenshot(
                    self.front_camera_frame, 
                    self.front_camera_path, 
                    self.file_number,
                    True if self.mode_current in self.modes_pid else False
                ),
                setattr(self, "wait_execute", False)
            )
        )

    def execute_force_mode(self):
        current_force_amt = (
            self.max_force if keyboard.is_pressed("space")
            else (self.dry_test_force if self.mode_current == self.mode_dry_test else self.pool_force)
        )

        desired_x_force = 0
        desired_y_force = 0
        desired_z_force = 0
        desired_x_torque = 0
        desired_y_torque = 0
        desired_z_torque = 0
        
        if keyboard.is_pressed("w"):
            desired_x_force += current_force_amt * self.MAX_FWD_FORCE
        if keyboard.is_pressed("s"):
            desired_x_force += current_force_amt * self.MAX_BKWD_FORCE
        if keyboard.is_pressed("a"):
            desired_y_force += current_force_amt * self.MAX_FWD_FORCE
        if keyboard.is_pressed("d"):
            desired_y_force += current_force_amt * self.MAX_BKWD_FORCE
        if keyboard.is_pressed("q"):
            desired_z_force += current_force_amt * self.MAX_FWD_FORCE
        if keyboard.is_pressed("e"):
            desired_z_force += current_force_amt * self.MAX_BKWD_FORCE
        if keyboard.is_pressed("o"):
            desired_x_torque += current_force_amt * self.MAX_FWD_FORCE
        if keyboard.is_pressed("u"):
            desired_x_torque += current_force_amt * self.MAX_BKWD_FORCE
        if keyboard.is_pressed("i"):
            desired_y_torque += current_force_amt * self.MAX_FWD_FORCE
        if keyboard.is_pressed("k"):
            desired_y_torque += current_force_amt * self.MAX_BKWD_FORCE
        if keyboard.is_pressed("j"):
            desired_z_torque += current_force_amt * self.MAX_FWD_FORCE
        if keyboard.is_pressed("l"):
            desired_z_torque += current_force_amt * self.MAX_BKWD_FORCE

        self.pub_x.publish(desired_x_force)
        self.pub_y.publish(desired_y_force)
        self.pub_z.publish(desired_z_force)
        self.pub_roll.publish(desired_x_torque)
        self.pub_pitch.publish(desired_y_torque)
        self.pub_yaw.publish(desired_z_torque)

    def execute_pid_mode(self):
        if self.mode_current == self.mode_pid_global:
            target_x = self.x
            target_y = self.y
            target_z = self.x
            target_quaternion = self.quaternion
        else:
            target_x = 0
            target_y = 0
            target_z = 0
            target_quaternion = np.quaternion(1, 0, 0, 0)

        if keyboard.is_pressed("w"): target_x += self.pid_position_delta
        if keyboard.is_pressed("s"): target_x -= self.pid_position_delta
        if keyboard.is_pressed("a"): target_y += self.pid_position_delta
        if keyboard.is_pressed("d"): target_y -= self.pid_position_delta
        if keyboard.is_pressed("q"): target_z += self.pid_position_delta
        if keyboard.is_pressed("e"): target_z -= self.pid_position_delta
        if keyboard.is_pressed("o"): 
            target_quaternion *= np.quaternion(
                self.pid_quaternion_delta_w,
                self.pid_quaternion_delta_xyz,
                0.0,
                0.0
            )
        if keyboard.is_pressed("u"): 
            target_quaternion *= np.quaternion(
                self.pid_quaternion_delta_w,
                -self.pid_quaternion_delta_xyz,
                0.0,
                0.0
            )
        if keyboard.is_pressed("i"): 
            target_quaternion *= np.quaternion(
                self.pid_quaternion_delta_w,
                0.0,
                self.pid_quaternion_delta_xyz,
                0.0
            )
        if keyboard.is_pressed("k"): 
            target_quaternion *= np.quaternion(
                self.pid_quaternion_delta_w,
                0.0,
                -self.pid_quaternion_delta_xyz,
                0.0
            )
        if keyboard.is_pressed("j"): 
            target_quaternion *= np.quaternion(
                self.pid_quaternion_delta_w,
                0.0,
                0.0,
                self.pid_quaternion_delta_xyz
            )
        if keyboard.is_pressed("l"): 
            target_quaternion *= np.quaternion(
                self.pid_quaternion_delta_w,
                0.0,
                0.0,
                -self.pid_quaternion_delta_xyz
            )

        if self.mode_current == self.mode_pid_local:
            # Convert local desired postion (target_[xyz]) to
            # NWU positon.
            target_x, target_y, target_z = local_to_global(
                self.quaternion,
                [target_x, target_y, target_z]
            )  
            # Convert local desired rotation (target_quaternion)  
            # to NWU rotation (self.quaternion).
            target_quaternion = self.quaternion * target_quaternion
        
        self.pub_pid_x.publish(Float64(target_x))
        self.pub_pid_y.publish(Float64(target_y))
        self.pub_pid_z.publish(Float64(target_z))
        self.pub_pid_quat.publish(
            Quaternion(
                target_quaternion.x, 
                target_quaternion.y, 
                target_quaternion.z,
                target_quaternion.w
            )
        )
        target_roll, target_pitch, target_yaw = transformations.euler_from_quaternion(
            [
                target_quaternion.w, 
                target_quaternion.x, 
                target_quaternion.y, 
                target_quaternion.z
            ]
        ) 
        pretty_print_pid([target_x, target_y, target_z, target_roll, target_pitch, target_yaw])

    def execute(self):
        self.establish_key_hooks()
        print_key_options_msg(self.modes)

        while not (rospy.is_shutdown() or self.is_esc_pressed):
            if self.wait_execute:
                rospy.sleep(2)
            current_time = rospy.get_time()
            if (
                any(x is None for x in [self.x, self.y, self.z, self.quaternion]) or 
                current_time - self.last_pose_received > self.missing_pose_interval
            ):
                if current_time - self.last_warning_time > self.missing_pose_interval:
                    self.last_warning_time = current_time 
                    rospy.logwarn("Missing pose values.")
            else:
                self.modes_execution[self.mode_current]() 


    def shut_down_thrusters(self):
        # @TO-DO(Felipe): Fix format print message.
        if self.mode_current in self.modes_pid:
            clean_pretty_print()
            print("\n" * 6) 
        self.pub_pid_x_enable.publish(Bool(False))
        self.pub_pid_y_enable.publish(Bool(False))
        self.pub_pid_z_enable.publish(Bool(False))
        self.pub_pid_quat_enable.publish(Bool(False))
        reset_cmd = ThrusterMicroseconds(
            [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
        )
        self.pub_microseconds.publish(reset_cmd)
        print("Safely shutting down thrusters.")

    

if __name__ == "__main__":
    rospy.init_node("joystick")
    joystick = Joystick()
    # Safely shut down the thrusters.
    rospy.on_shutdown(joystick.shut_down_thrusters)  
    joystick.execute()

        

        

        

        
