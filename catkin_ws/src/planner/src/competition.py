#!/usr/bin/env python3

import rospy
import smach
from std_msgs.msg import String

from substates.breadth_first_search import *
from substates.in_place_search import *
from substates.linear_search import *
from substates.navigate_lane_marker import *
from substates.utility.controller import *
from substates.utility.state import *
from substates.utility.vision import *
from substates.trick import *
from substates.navigate_gate import *
from substates.navigate_buoy import *
from substates.octagon_task import *


class Competition:
     def __init__(self):
          self.mapping = ObjectMapper()
          self.state = StateTracker()
          self.control = Controller(rospy.Time(0))

          self.pub_mission_display = rospy.Publisher("/mission_display", String, queue_size=1)


     def gate(self):
          global sm
          self.pub_mission_display.publish("Gate")
          

     def lane_marker(self):
          global sm
          self.pub_mission_display.publish("Lane")
          
     
     def buoy(self):
          global sm
          self.pub_mission_display.publish("Buoy")  


     def dropper(self):
          global sm
          self.pub_mission_display.publish("Dropper")


     def octagon(self):
          global sm
          self.pub_mission_display.publish("Octagon")


     def torpedo(self):
          global sm
          self.pub_mission_display.publish("Torpedo")
     

     def tricks(self):
          global sm
          self.pub_mission_display.publish("Tricks")    
          

     def 


if __name__ == '__main__':
     rospy.init_node("competition_planner", log_level=rospy.DEBUG)

     mission_wait_time = rospy.get_param("mission_wait_time")

     try:
          sm = None
          competition = Competition()

          print(f"Waiting {mission_wait_time} seconds before starting mission...")
          rospy.sleep(mission_wait_time)
          print("Starting mission")

          

     except KeyboardInterrupt:
          #ASSUMING ONE CURRENTLY RUNNING STATE MACHINE AT A TIME (NO THREADS)
          if sm is not None: sm.request_preempt()