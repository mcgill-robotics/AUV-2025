#!/usr/bin/env python3
import smach
import rospy
import threading
from std_msgs.msg import String


class NavigateOctagon(smach.State):
    def __init__(self, control, mapping, state):
        super().__init__(outcomes=["success", "failure", "timeout"])
        self.control = control
        self.mapping = mapping
        self.state = state

        self.thread_timer = None
        self.timeout_occurred = False
        self.time_limit = rospy.get_param("octagon_time_limit")

        self.pub_mission_display = rospy.Publisher(
            "/mission_display", String, queue_size=1
        )
    
    def timer_thread_func(self):
        self.pub_mission_display.publish("Gate Time-out")
        self.timeout_occurred = True
        self.control.freeze_pose()

    def execute(self, ud):
        print("Starting octagon navigation.")
        self.pub_mission_display.publish("Octagon")

        # Start the timer in a separate thread.
        self.thread_timer = threading.Timer(self.time_limit, self.timer_thread_func)
        self.thread_timer.start()

        # MOVE TO MIDDLE OF POOL DEPTH AND FLAT ORIENTATION
        self.control.move((None, None, rospy.get_param("down_cam_search_depth")))
        self.control.flatten()

        auv_current_position = (self.state.x, self.state.y)

        octagon_obj = self.mapping.getClosestObject(
            "Octagon Table", (auv_current_position[0], auv_current_position[1])
        )

        if octagon_obj is None:
            print("No octagon in object map! Failed.")
            return "failure"

        if self.timeout_occurred:
            return "timeout"
        print("Moving to the center of the octagon.")
        self.control.move(
            (octagon_obj[1], octagon_obj[2], rospy.get_param("down_cam_search_depth")),
            face_destination=True,
        )
        print("Surfacing.")
        self.control.kill()

        print("Successfully navigated the octagon.")
        return "success"