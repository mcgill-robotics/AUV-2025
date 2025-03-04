#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from thrust_mapper_utils import *
from auv_msgs.msg import ThrusterMicroseconds
from geometry_msgs.msg import Wrench

import time

force_amt = 0.1  # 10%

# TODO: update if necessary
reset = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]

class ThrusterTest(Node):
    def __init__(self):
        super().__init__('thruster_test')
        self.pub = self.create_publisher(ThrusterMicroseconds, '/propulsion/microseconds', 1)
        self.effort_pub = self.create_publisher(Wrench, '/controls/effort', 1)
        self.reset_cmd = ThrusterMicroseconds(microseconds=reset)
        time.sleep(2)

    def simultaneous_forwards_test(self):
        while rclpy.ok():
            self.get_logger().info("- spinning at " + str(100 * force_amt) + "% max forwards force for 1s")

            cmd = ThrusterMicroseconds(microseconds=[force_to_pwm(force_amt * MAX_FWD_FORCE)] * 8)
            self.pub.publish(cmd)
            time.sleep(1.0)
            self.pub.publish(self.reset_cmd)

            choice = input("1. repeat test\n2. proceed\n")
            if choice != "1":
                break


    # just type the thruster you want and it will run
    def optimized_dry_test(self, t):
        self.get_logger().info("- spinning at " + str(100 * force_amt) + "% max forwards force for 1s")
        cmd = reset.copy()
        cmd[t - 1] = force_to_pwm(force_amt * MAX_FWD_FORCE * thruster_mount_dirs[t-1])
        self.pub.publish(ThrusterMicroseconds(microseconds=cmd))
        time.sleep(1.0)
        self.pub.publish(self.reset_cmd)

    def reset_thrusters(self):
        self.pub.publish(self.reset_cmd)
        self.get_logger().info("Safely shutting down thrusters")

    def re_arm(self):
        time.sleep(1)
        msg1 = ThrusterMicroseconds(microseconds=[1500] * 8)
        msg2 = ThrusterMicroseconds(microseconds=[1540] * 8)

        self.pub.publish(msg1)
        time.sleep(0.5)
        self.pub.publish(msg2)
        time.sleep(0.5)
        self.pub.publish(msg1)


    def keyboard_control(self):
        self.get_logger().info("NOTE: Uses thrust mapper")
        self.get_logger().info(" > WASD for SURGE/SWAY\n > Q/E for UP/DOWN\n > IJKL for PITCH/YAW\n > U/O for ROLL\n > ESC to exit")
        
        while rclpy.ok():
            desired_effort = Wrench()
            desired_effort.force.x = 0.0
            desired_effort.force.y = 0.0
            desired_effort.force.z = 0.0
            desired_effort.torque.x = 0.0
            desired_effort.torque.y = 0.0
            desired_effort.torque.z = 0.0
            
            # Prompt the user for input
            user_input = input("Enter a command (WASD/QE/IJKL/UO/ESC to exit): ").lower()

            # Check for exit condition
            if user_input == "esc":
                break

            # Handle the user input and adjust forces and torques accordingly
            if user_input == "w":
                desired_effort.force.x += force_amt * MAX_FWD_FORCE
            elif user_input == "s":
                desired_effort.force.x += force_amt * MAX_BKWD_FORCE
            elif user_input == "a":
                desired_effort.force.y += force_amt * MAX_FWD_FORCE
            elif user_input == "d":
                desired_effort.force.y += force_amt * MAX_BKWD_FORCE
            elif user_input == "q":
                desired_effort.force.z += force_amt * MAX_FWD_FORCE
            elif user_input == "e":
                desired_effort.force.z += force_amt * MAX_BKWD_FORCE
            elif user_input == "o":
                desired_effort.torque.x += force_amt * MAX_FWD_FORCE
            elif user_input == "u":
                desired_effort.torque.x += force_amt * MAX_BKWD_FORCE
            elif user_input == "i":
                desired_effort.torque.y += force_amt * MAX_FWD_FORCE
            elif user_input == "k":
                desired_effort.torque.y += force_amt * MAX_BKWD_FORCE
            elif user_input == "j":
                desired_effort.torque.z += force_amt * MAX_FWD_FORCE
            elif user_input == "l":
                desired_effort.torque.z += force_amt * MAX_BKWD_FORCE
            else:
                self.get_logger().info("Invalid input. Please enter a valid command.")

            # Publish the desired effort
            self.effort_pub.publish(desired_effort)


    def run_tests(self):
        while rclpy.ok():
            self.get_logger().info("========== Thrusters Test ==========")
            self.get_logger().info("refer to images in dry-test/images for labelled diagrams of thrusters on the AUV")

            self.get_logger().info("1. select thruster to test\n2. test thrusters simultaneously\n3. re-arm\n4. Keyboard control\n5. exit\n")

            choice = input("Please choose an option (1-5): ")

            if choice == "1":
                while rclpy.ok():
                    choice = int(input("select thruster to test (1-8): "))
                    if 1 <= choice and choice <= 8:
                        self.optimized_dry_test(choice)
                    else:
                        break

            elif choice == "2":
                self.simultaneous_forwards_test()

            elif choice == "3":
                self.re_arm()
            
            elif choice == "4":
                self.keyboard_control()

            else:
                break


def main(args=None):
    rclpy.init(args=args)
    
    node = ThrusterTest()

    try:
        node.run_tests()
    except KeyboardInterrupt:
        pass
    finally:
        node.reset_thrusters()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
    