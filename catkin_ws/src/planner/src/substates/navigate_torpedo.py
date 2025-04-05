#!/usr/bin/env python3
import rospy
import smach
from .utility.functions import *
import threading
from std_msgs.msg import String

# imports for object detection
import cv2
from PIL import Image
import numpy as np

class NavigateTorpedo(smach.State):
    def __init__(self, control, mapping, state):
        super().__init__(outcomes=["success", "failure", "timeout"])
        self.control = control
        self.mapping = mapping
        self.state = state

        self.thread_timer = None
        self.timeout_occurred = False
        self.time_limit = rospy.get_param("torpedo_time_limit") # reads params from ros params
        self.centering_dist_threshold = rospy.get_param("center_dist_threshold")
        self.centering_delta_increment = rospy.get_param("centering_delta_increment")
        
        # Creates a ROS publisher to send status updates to the /mission_display topic
        self.pub_mission_display = rospy.Publisher(
            "/mission_display", String, queue_size=1
        )

    def timer_thread_func(self):
        self.pub_mission_display.publish("Torpedo Time-out")
        self.timeout_occurred = True
        self.control.freeze_pose()

    # # implement colour detection helper function
    # # in open cv, you can maybe give a name to a camera
    # def get_limits(color):
    #     c = np.uint8([[color]])  # BGR values
    #     hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

    #     hue = hsvC[0][0][0]  # Get the hue value

    #     # Handle red hue wrap-around
    #     if hue >= 165:  # Upper limit for divided red hue
    #         lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
    #         upperLimit = np.array([180, 255, 255], dtype=np.uint8)
    #     elif hue <= 15:  # Lower limit for divided red hue
    #         lowerLimit = np.array([0, 100, 100], dtype=np.uint8)
    #         upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)
    #     else:
    #         lowerLimit = np.array([hue - 10, 100, 100], dtype=np.uint8)
    #         upperLimit = np.array([hue + 10, 255, 255], dtype=np.uint8)

    #     return lowerLimit, upperLimit
    
    # def detect_target(self):
    #     RED = [0, 0, 255]  # red in BGR colorspace
    #     cap = cv2.VideoCapture(0)

    #     lowerLimit, upperLimit = self.get_limits(color=RED)

    #     while True:
    #         ret, frame = cap.read()

    #         hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #         mask = cv2.inRange(hsvImage, lowerLimit, upperLimit)

    #         mask_ = Image.fromarray(mask)

    #         bbox = mask_.getbbox()
            
    #         if bbox is not None:
    #             x1, y1, x2, y2 = bbox

    #             if (x2 - x1) > 10 and (y2 - y1) > 10:
    #                 frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)

    #         cv2.imshow('frame', frame)

    #         if cv2.waitKey(1) & 0xFF == ord('q'):
    #             break

    #     cap.release()

    #     cv2.destroyAllWindows()


    # implement centering of torpedo helper 

    def execute(self, ud):
        print("Starting Torpedo Navigation")
        self.pub_mission_display.publish("Torpedo")

        # Start the timer in a separate thread.
        self.thread_timer = threading.Timer(self.time_limit, self.timer_thread_func)
        self.thread_timer.start()

        # Move to the middle of the pool depth and flat orientation.
        self.control.flatten()

        # Find red targets
        target = self.mapping.getClosestObject(cls="Red Target", pos=(self.state.x, self.state.y))

        if target is None:
            print("No target detected! Failed.")
            return "failure"
        
        print("Moving to align with target")
        if self.timeout_occurred:
            return "timeout"

        # Move toward target's position
        # !!!! add depths where the torpedo should be fired !!!
        self.control.move((target[1], target[2], rospy.get_param("front_cam_search_depth")), face_destination=True)

        # Centering loop
        while self.mapping.distance > self.centering_dist_threshold:
            if self.timeout_occurred:
                return "timeout"
            
            if self.mapping.delta_height < 0:
                self.control.moveDeltaLocal((self.centering_delta_increment, 0, 0))
            elif self.mapping.delta_height > 0:
                self.control.moveDeltaLocal((-self.centering_delta_increment, 0, 0))

            if self.mapping.delta_width < 0:
                self.control.moveDeltaLocal((0, self.centering_delta_increment, 0))
            elif self.mapping.delta_width > 0:
                self.control.moveDeltaLocal((0, -self.centering_delta_increment, 0))

            rospy.sleep(3)

        print("Target Centered. Preparing to fire.")

        # Ensure AUV is properly oriented
        if self.timeout_occurred:
            return "timeout"
        self.control.flatten()

        # Fire the torpedo
        if self.timeout_occurred:
            return "timeout"
        print("Firing torpedo!")
        self.control.fire_torpedo()  # connect fire torpedo here 
        rospy.sleep(1)
        print("Torpedo launched successfully.")
        return "success"
            