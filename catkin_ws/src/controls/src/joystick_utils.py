import os
import numpy as np
import quaternion
import cv2
from cv_bridge import CvBridge

# Pretty print PID parameters.
max_width = 5
terminal_cursor_up = "\033[A"
horizontal_box = chr(0x2550)
vertical_box = chr(0x2551)
top_left_box = chr(0x2554)
top_right_box = chr(0x2557)
bottom_left_box = chr(0x255A)
bottom_right_box = chr(0x255D)
space_indentation = " " * 11


def print_key_options_msg(modes): 
     print("NOTE: Launch controls.launch and propulsion.launch to use joystick.")
     print(" > WASD for SURGE/SWAY.")
     print(" > Q/E for UP/DOWN.")
     print(" > IJKL for PITCH/YAW.")
     print(" > U/O for ROLL.")
     print(f" > To switch to [{modes[0]}] mode, press 0.")
     print(f" > To switch to [{modes[1]}] mode, press 1.")
     print(f" > To switch to [{modes[1]}] mode, press 2.")
     print(f" > To switch to [{modes[2]}] mode, press 3.")
     print(" > For down camera screenshot, press 4.")
     print(" > For front camera screenshot, press 5.")
     print(f" > CURRENT MODE: [{modes[0]}].")
     print(" > Hold SPACE for max force.")

def format_number(number, max_width):
     formatted_number = f"{abs(number):.4f}"[:max_width].rjust(max_width, "0")
     return f"{'-' if number < 0 else ' '}{formatted_number}"

def pretty_print_pid(targets):
     x, y, z, roll, pitch, yaw = [format_number(num, max_width) for num in targets]
     print(f"{top_left_box}{horizontal_box*8}Current PID setpoint{horizontal_box*8}{top_right_box}")
     print(f"{vertical_box}{space_indentation}    x = {x}{space_indentation}{vertical_box}")
     print(f"{vertical_box}{space_indentation}    y = {y}{space_indentation}{vertical_box}")
     print(f"{vertical_box}{space_indentation}    z = {z}{space_indentation}{vertical_box}")
     print(f"{vertical_box}{space_indentation} roll = {roll}{space_indentation}{vertical_box}")
     print(f"{vertical_box}{space_indentation}pitch = {pitch}{space_indentation}{vertical_box}")
     print(f"{vertical_box}{space_indentation}  yaw = {yaw}{space_indentation}{vertical_box}")
     print(f"{bottom_left_box}{horizontal_box*36}{bottom_right_box}", end="\r")
     print(terminal_cursor_up * 7, end="\r")   

def clean_pretty_print():
     for _ in range(8):
          print(" "*38)
     print(terminal_cursor_up * 8, end="\r")   

def take_screenshot(image, path, file_number, clean_print):
     if image is None:
          rospy.logwarn("No frame has been received from camera! Screenshot failed.")
          return
     bridge = CvBridge()
     cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
     file_path = os.path.join(path, f"image_{file_number}.jpg")
     cv2.imwrite(file_path, cv_image)
     if clean_print:
          clean_pretty_print()
     print(f"📸 Cheese! Camera picture was taken successfully and saved to {path}")

def local_to_global(current_quaternion, target_position):
     pivot_quat = current_quaternion
     global_goal = (
          pivot_quat
          * np.quaternion(0, target_position[0], target_position[1], target_position[2])
          * pivot_quat.inverse()
     )
     return [global_goal.x, global_goal.y, global_goal.z]


# Automatically import all functions and constants.
__all__ = [name for name in globals() if not name.startswith('__')]