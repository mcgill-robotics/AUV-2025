#!/usr/bin/env python3

import rospy
import keyboard
from tf import transformations
import numpy as np
import quaternion
import cv2
import os
from cv_bridge import CvBridge

from auv_msgs.msg import ThrusterMicroseconds
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Quaternion, Pose
from sensor_msgs.msg import Image


class Joystick:
    def __init__(self) -> None:
        self.MAX_FWD_FORCE = 4.52 * 9.81
        self.MAX_BKWD_FORCE = -3.52 * 9.81
        self.joystick_max_force = float(rospy.get_param("joystick_max_force"))
        self.joystick_dry_test_force = float(rospy.get_param("joystick_dry_test_force"))

        self.missing_pose_interval = float(rospy.get_param("joystick_missing_pose_warning_interval"))
        self.last_warning_time = rospy.Time.now()
        
        # Mode names.
        self.mode_force = "FORCE"
        self.mode_pid_global = "PID GLOBAL"
        self.mode_pid_local = "PID LOCAL"
        self.mode_current = self.mode_force

        self.modes_execution = {
            self.mode_force :  self.execute_force_mode,
            self.mode_pid_local : self.execute_pid_mode,
            self.mode_pid_global : self.execute_pid_mode
        }

        self.is_esc_pressed = False      

        self.folder_path = rospy.get_param("joystick_image_folder_path")
        self.down_camera_path = os.path.join(self.folder_path, "down_camera")
        self.front_camera_path = os.path.join(self.folder_path, "front_camera")
        self.file_number = 1
        
        # Ensure the folder exists.
        os.makedirs(self.folder_path, exist_ok=True)
        os.makedirs(self.down_camera_path, exist_ok=True)
        os.makedirs(self.front_camera_path, exist_ok=True)

        self.down_camera_frame = None
        self.front_camera_frame = None

        self.pid_position_delta = rospy.get_param("joystick_pid_position_delta")   
        self.pid_quaternion_delta_w = rospy.get_param("joystick_pid_quaternion_delta_w")   
        self.pid_quaternion_delta_xyz = rospy.get_param("joystick_pid_quaternion_delta_xyz")   

        self.x = None
        self.y = None
        self.z = None
        self.quaternion = None
        
        self.max_width = 5
        self.terminal_cursor_up = "\033[A"
        self.horizontal_box = chr(0x2550)
        self.vertical_box = chr(0x2551)
        self.top_left_box = chr(0x2554)
        self.top_right_box = chr(0x2557)
        self.bottom_left_box = chr(0x255A)
        self.bottom_right_box = chr(0x255D)
        self.space_indentation = " " * 11

        rospy.Subscriber("/state/pose", Pose, self.cb_pose)
        rospy.Subscriber("/vision/down_cam/image_raw", Image, self.cb_down_camera)
        rospy.Subscriber("/vision/front_cam/color/image_raw",
            Image, self.cb_front_camera
        )


    def cb_pose(self, msg):
        self.x = msg.position.x
        self.y = msg.position.y
        self.z = msg.position.z
        self.quaternion = np.quaternion(
            x=msg.orientation.x,
            y=msg.orientation.y,
            z=msg.orientation.z,
            w=msg.orientation.w
        )

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
        
    def establish_force_publishers(self) -> None:
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

    def establish_pid_enable_publishers(self) -> None:
        # [PID] mode.
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
   
    def establish_pid_publishers(self) -> None:
        # [PID] mode.
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

    def set_enable_pid(self, enable):
        self.pub_pid_x_enable.publish(Bool(enable))
        self.pub_pid_y_enable.publish(Bool(enable))
        self.pub_pid_z_enable.publish(Bool(enable))
        self.pub_pid_quat_enable.publish(Bool(enable))

    def take_screenshot(self, image, path):
        if image is None:
            rospy.logwarn("No frame has been received from camera! Screenshot failed.")
            return
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="passthrough")
        file_path = os.path.join(path, f"image_{self.file_number}.jpg")
        cv2.imwrite(file_path, cv_image)
        self.file_number += 1

    def establish_key_hooks(self) -> None:
        mode_swap = lambda mode, enable: (
            setattr(self, "mode_current", mode),
            self.set_enable_pid(enable)
        )
        keyboard.on_press_key(
            "esc", 
            lambda _: ( 
                setattr(self, "is_esc_pressed", True)
            )
        )
        keyboard.on_press_key(
            "1", 
            mode_swap(self.mode_force, False)
        )
        keyboard.on_press_key(
            "2", 
            mode_swap(self.mode_pid_local, True)
        )
        keyboard.on_press_key(
            "3", 
            mode_swap(self.mode_pid_global, True)
        )
        keyboard.on_press_key(
            "4",
            self.take_screenshot(self.down_camera_frame, self.down_camera_path)
        )
        keyboard.on_press_key(
            "5",
            self.take_screenshot(self.front_camera_frame, self.front_camera_path)
        )


    def print_key_options_msg(self) -> None: 
        print("NOTE: Launch controls.launch and propulsion.launch to use joystick.")
        print(" > WASD for SURGE/SWAY")
        print(" > Q/E for UP/DOWN")
        print(" > IJKL for PITCH/YAW")
        print(" > U/O for ROLL")
        print(" > Hold SPACE for max. force")
        print(f" > To switch to [{self.mode_force}] mode, press 1")
        print(f" > To switch to [{self.mode_pid_local}] mode, press 2")
        print(f" > To switch to [{self.mode_pid_global}] mode, press 3")
        print(f" > CURRENT MODE: [{self.mode_current}]")
        print(" > For down camera screenshot, press 4")
        print(" > For front camera screenshot, press 5")


    def print_pid_values(self, targets):
        x, y, z, roll, pitch, yaw = [self.format_number(num) for num in targets]
        print(f"{self.self.top_left_box}{self.horizontal_box*8}Current PID setpoint{self.horizontal_box*8}{self.top_right_box}")
        print(f"{self.vertical_box}{self.space_indentation}    x = {x}{self.space_indentation}{self.vertical_box}")
        print(f"{self.vertical_box}{self.space_indentation}    y = {y}{self.space_indentation}{self.vertical_box}")
        print(f"{self.vertical_box}{self.space_indentation}    z = {z}{self.space_indentation}{self.vertical_box}")
        print(f"{self.vertical_box}{self.space_indentation} roll = {roll}{self.space_indentation}{self.vertical_box}")
        print(f"{self.vertical_box}{self.space_indentation}pitch = {pitch}{self.space_indentation}{self.vertical_box}")
        print(f"{self.vertical_box}{self.space_indentation}  yaw = {yaw}{self.space_indentation}{self.vertical_box}")
        print(f"{self.bottom_left_box}{self.horizontal_box*36}{self.bottom_right_box}", end="\r")
        print(self.terminal_cursor_up * 7, end="\r")

    def format_number(self, number):
        formatted_number = f"{abs(number):.4f}"[:self.max_width].rjust(self.max_width, "0")
        return f"{'-' if number < 0 else ' '}{formatted_number}"

    def reset_thrusters(self) -> None:
        reset_cmd = ThrusterMicroseconds(
            [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
        )
        self.pub_microseconds.publish(reset_cmd)
        print("Safely shutting down thrusters")
    
    def local_to_global(self, target_position):
        pivot_quat = self.quaternion
        global_goal = (
            pivot_quat
            * np.quaternion(0, target_position[0], target_position[1], target_position[2])
            * pivot_quat.inverse()
        )
        return [global_goal.x, global_goal.y, global_goal.z]

    def execute_force_mode(self) -> None:
        current_force_amt = (
            self.joystick_max_force
            if keyboard.is_pressed("space")
            else self.joystick_dry_test_force
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

    def execute_pid_mode(self) -> None:
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
            target_x, target_y, target_z = self.local_to_global(
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
                w=target_quaternion.w, 
                x=target_quaternion.x, 
                y=target_quaternion.y, 
                z=target_quaternion.z
            )
        )
        
        target_roll, target_pitch, target_yaw = transformations.euler_from_quaternion(
            target_quaternion.w,
            target_quaternion.z,
            target_quaternion.y,
            target_quaternion.x
        ) 
        targets = [target_x, target_y, target_z, target_roll, target_pitch, target_yaw]
        self.print_pid_values(targets)
        
        
    def execute(self) -> None:
        self.establish_force_publishers()
        self.establish_pid_publishers()
        self.establish_pid_enable_publishers()
        self.establish_key_hooks()
        self.print_key_options_msg()

        while not (rospy.is_shutdown() or self.is_esc_pressed):
            if self.mode_current != self.mode_force:
                if any(x is None for x in [self.x, self.y, self.z, self.quaternion]):
                    if rospy.Time.now() - self.last_warning_time > self.missing_pose_interval:
                        rospy.logwarn("Missing pose values.")
                        continue
            self.modes_execution[self.mode_current]() 

    

if __name__ == "__main__":
    rospy.init_node("joystick")
    joystick = Joystick()
    # Safely shut down the thrusters.
    rospy.on_shutdown(joystick.reset_thrusters)  
    joystick.execute()

        

        

        

        
