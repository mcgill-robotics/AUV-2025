#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
import os
import keyboard

from sensor_msgs.msg import Image

from datetime import datetime

def front_cam_image_callback(msg):
    global front_cam_cur_image
    try:
        front_cam_cur_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error (front cam): {e}")
    # global front_cam_cur_image
    # front_cam_cur_image = bridge.imgmsg_to_cv2(msg, "bgr8")

def down_cam_image_callback(msg):
    global down_cam_cur_image
    try:
        down_cam_cur_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(f"CvBridge Error (down cam): {e}")
    # global down_cam_cur_image
    # down_cam_cur_image = bridge.imgmsg_to_cv2(msg, "bgr8")

def save_image(output_dir, is_front_cam):
    if is_front_cam:
        cur_image = front_cam_cur_image
    else:
        cur_image = down_cam_cur_image

    ros_time = rospy.Time.now()
    stamp = ros_time.to_sec()
    time_str = datetime.fromtimestamp(stamp).strftime("%Y-%m-%d_%H-%M-%S")

    filename = os.path.join(output_dir, f"image_{time_str}.jpg")
    cv2.imwrite(filename, cur_image)
    print(f'{"Front cam" if is_front_cam else "Down cam"} image saved')


if __name__ == "__main__":
    rospy.init_node("image_collection")

    bridge = CvBridge()
    FRONT_CAM_DATA_DIR = rospy.get_param("image_data_dir/front_cam")
    DOWN_CAM_DATA_DIR = rospy.get_param("image_data_dir/down_cam")
    if not os.path.exists(FRONT_CAM_DATA_DIR):
        os.makedirs(FRONT_CAM_DATA_DIR)
    if not os.path.exists(DOWN_CAM_DATA_DIR):
        os.makedirs(DOWN_CAM_DATA_DIR)

    front_cam_cur_image, down_cam_cur_image = None, None

    front_cam_image_sub = rospy.Subscriber(
        "/zed2i/zed_node/stereo/image_rect_color", Image, front_cam_image_callback
    )
    down_cam_image_sub = rospy.Subscriber(
        "/vision/down_cam/image_raw", Image, down_cam_image_callback
    )

    rospy.sleep(10) #TODO: add loop that checks if images ar being published

    while not rospy.is_shutdown():
        camera_usr_choice = input("\n\nChoose camera [0] if front, [1] if down ")
        if camera_usr_choice == "0":
            chosen_cam = True
        elif camera_usr_choice == "1":
            chosen_cam = False
        else: 
            print("Not a valid option!!!")
            continue
        
        SAVE_DIR = FRONT_CAM_DATA_DIR if chosen_cam else DOWN_CAM_DATA_DIR
        
        while True: 
            capture_method_usr_choice = input("\n-To take manual screen shots, press [z]\n-To take automatic screenshots, press [c],\n-To go back to choosing the camera, press [b]: ")

            if capture_method_usr_choice == "c":
                
                while True: 
                    delay_choice = input("\n-To go back to choosing the camera capturing method press [b].\n-If you want to continue, enter time delay (seconds): ")
                    
                    if not delay_choice.strip():
                        print("\nEmpty input! Please enter a number or press [b] to go back: ")
                        continue

                    if delay_choice == "b":
                        break

                    try:
                        delay_choice = int(delay_choice)
                        # It's a valid int
                    except ValueError:
                        # Not an int
                        print("\nThe time delay is not an integer.")
                        continue

                    if delay_choice < 0:
                        print("\nThe time delay should be a positive integer.")
                        continue
                    
                    print("\nSpam [p] to cancel")

                    while True:
                        save_image(SAVE_DIR, chosen_cam)
                        rospy.sleep(delay_choice)

                        if keyboard.is_pressed('p'):
                            print("Stopping capture...")
                            break

            elif capture_method_usr_choice == "z":
                save_image(SAVE_DIR, chosen_cam)
            elif capture_method_usr_choice == "b":
                break
            else: 
                print("\nNot a valid option!!!")
