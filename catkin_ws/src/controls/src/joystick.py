#!/usr/bin/env python3

import rospy
import keyboard
import math
from tf import transformations
from auv_msgs.msg import ThrusterMicroseconds
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Quaternion, Pose



# Forces produced by T200 thruster at 14V (N).



class Joystick:
    def __init__(self) -> None:
        self.MAX_FWD_FORCE = 4.52 * 9.81
        self.MAX_BKWD_FORCE = -3.52 * 9.81
        
        # Mode names.
        self.mode_force = "FORCE"
        self.mode_pid_global = "PID GLOBAL"
        self.mode_pid_local = "PID LOCAL"
        self.current_mode = self.mode_force

        self.modes_execution = {
            self.mode_force :  self.execute_force_mode,
            self.mode_pid_local : self.execute_pid_local_mode,
            self.mode_pid_global : self.execute_pid_global_mode
        }

        self.is_esc_pressed = False      

        self.pid_position_delta = rospy.get_param("joystick_pid_position_delta")   
        self.pid_orientation_delta = rospy.get_param("joystick_pid_orientation_delta")   

        self.x = None
        self.y = None
        self.z = None
        self.theta_x = None
        self.theta_y = None
        self.theta_z = None
        self.orientation = None

        rospy.Subscriber("/state/pose", Pose, self.callback_pose)
        rospy.Subscriber("/state/theta/x", Float64, self.callback_theta_x)
        rospy.Subscriber("/state/theta/y", Float64, self.callback_theta_y)
        rospy.Subscriber("/state/theta/z", Float64, self.callback_theta_z)

    def callback_pose(self, msg):
        self.x = msg.position.x
        self.y = msg.position.y
        self.z = msg.position.z
        self.orientation = msg.orientation

    def callback_theta_x(self, msg):
        self.theta_x = msg.data

    def callback_theta_y(self, msg):
        self.theta_y = msg.data

    def callback_theta_z(self, msg):
        self.theta_z = msg.data
        
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
        self.pub_pid_x_enable(Bool(enable))
        self.pub_pid_y_enable(Bool(enable))
        self.pub_pid_z_enable(Bool(enable))
        self.pub_pid_quat_enable(Bool(enable))

    def establish_key_hooks(self) -> None:
        mode_swap = lambda mode, enable: (
            setattr(self, "current_mode", mode),
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

    def print_options_msg(self) -> None: 
        print("NOTE: Launch controls.launch and propulsion.launch to use joystick.")
        print(" > WASD for SURGE/SWAY")
        print(" > Q/E for UP/DOWN")
        print(" > IJKL for PITCH/YAW")
        print(" > U/O for ROLL")
        print(" > hold SPACE for max. force")
        print(f" > To switch to [{self.mode_force}] mode, press 1")
        print(f" > To switch to [{self.local_mode}] mode, press 2")
        print(f" > To switch to [{self.mode_pid_global}] mode, press 3")
        print(f" > CURRENT MODE: [{self.current_mode}]")

    def reset_thrusters(self) -> None:
        reset_cmd = ThrusterMicroseconds(
            [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]
        )
        self.pub_microseconds.publish(reset_cmd)
        print("Safely shutting down thrusters")

    def euler_to_quaternion(self, roll, pitch, yaw):
        # Returns [w, x, y, z]
        q = transformations.quaternion_from_euler(
            math.pi * roll / 180, math.pi * pitch / 180, math.pi * yaw / 180, "rxyz"
        )
        return [q[3], q[0], q[1], q[2]]

    def execute_force_mode(self) -> None:
        current_force_amt = (
            float(rospy.get_param("joystick_max_force"))
            if keyboard.is_pressed("space")
            else float(rospy.get_param("joystick_dry_test_force"))
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

    def execute_pid_local_mode(self) -> None:
        target_x = self.x
        target_y = self.y
        target_z = self.x
        target_theta_x = self.theta_x
        target_theta_y = self.theta_y
        target_theta_z = self.theta_z

        if keyboard.is_pressed("w"): target_x += self.pid_position_delta
        if keyboard.is_pressed("s"): target_x -= self.pid_position_delta
        if keyboard.is_pressed("a"): target_y += self.pid_position_delta
        if keyboard.is_pressed("d"): target_y -= self.pid_position_delta
        if keyboard.is_pressed("q"): target_z += self.pid_position_delta
        if keyboard.is_pressed("e"): target_z -= self.pid_position_delta
        if keyboard.is_pressed("o"): target_theta_x += self.pid_orientation_delta
        if keyboard.is_pressed("u"): target_theta_x -= self.pid_orientation_delta
        if keyboard.is_pressed("i"): target_theta_y += self.pid_orientation_delta
        if keyboard.is_pressed("k"): target_theta_y -= self.pid_orientation_delta
        if keyboard.is_pressed("j"): target_theta_z += self.pid_orientation_delta
        if keyboard.is_pressed("l"): target_theta_z -= self.pid_orientation_delta

        quat = self.euler_to_quaternion(target_theta_x, target_theta_y, target_theta_z)
        self.pub_pid_x.publish(Float64(target_x))
        self.pub_pid_y.publish(Float64(target_y))
        self.pub_pid_z.publish(Float64(target_z))
        self.pub_pid_quat.publish(
            Quaternion(w=quat[0], x=quat[1], y=quat[2], z=quat[2])
        )

    def execute_pid_global_mode(self) -> None:
        pass
        
    def execute(self) -> None:
        self.establish_force_publishers()
        self.establish_pid_publishers()
        self.establish_pid_enable_publishers()
        self.establish_key_hooks()

        while not rospy.is_shutdown() and self.is_esc_pressed:
            self.modes_execution[self.current_mode]() 

    

if __name__ == "__main__":
    rospy.init_node("joystick")
    joystick = Joystick()
    # To safely shut down the thrusters.
    rospy.on_shutdown(joystick.reset_thrusters)  
    joystick.execute()

        

        

        

        
