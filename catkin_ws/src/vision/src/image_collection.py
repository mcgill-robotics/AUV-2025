#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
import os

from sensor_msgs.msg import Image


def front_cam_image_callback(msg):
    global front_cam_cur_image
    front_cam_cur_image = bridge.imgmsg_to_cv2(msg, "bgr8")

def down_cam_image_callback(msg):
    global down_cam_cur_image
    down_cam_cur_image = bridge.imgmsg_to_cv2(msg, "bgr8")

def save_image(output_dir, is_front_cam):
    if is_front_cam:
        cur_image = front_cam_cur_image
    else:
        cur_image = down_cam_cur_image

    filename = os.path.join(output_dir, "image_{}.jpg".format(rospy.Time.now()))
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

    while not rospy.is_shutdown():
        usr_choice = input("-To take manual screen shots, press [z]\n-To take automatic screenshots, press [c] ")
        chosen_frontcam = True if input("Choose camera [0] if front, [1]] if down ") == "0" else False  
        SAVE_DIR = FRONT_CAM_DATA_DIR if chosen_frontcam else DOWN_CAM_DATA_DIR
        time_delay = int(input("Enter time delay (seconds): ")) if usr_choice == "c" else 0

        if usr_choice == "c":
            print("Spam [p] to cancel")

            while True:
                save_image(SAVE_DIR, chosen_frontcam)
                rospy.sleep(time_delay)

                if keyboard.is_pressed('p'):
                    print("Stopping capture...")
                    break

        elif usr_choice == "z":
            save_image(SAVE_DIR, chosen_frontcam)
        else:
            print("Not a valid option!!!")