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


class Missions:
     def __init__(self, master_planner):
          self.master_planner = master_planner
          
          self.mapping = ObjectMapper()
          self.state = StateTracker()
          self.control = Controller(rospy.Time(0))

          self.coin_flip_time_limit = rospy.get_param("coin_flip_time_limit")
          self.gate_time_limit = rospy.get_param("gate_time_limit")
          self.lane_marker_time_limit = rospy.get_param("lane_marker_time_limit")
          self.tricks_time_limit = rospy.get_param("tricks_time_limit")
          self.buoy_time_limit = rospy.get_param("buoy_time_limit")
          self.dropper_time_limit = rospy.get_param("dropper_time_limit")
          self.torpedo_time_limit = rospy.get_param("torpedo_time_limit")
          self.octagon_time_limit = rospy.get_param("octagon_time_limit")
          self.mission_wait_time = rospy.get_param("mission_wait_time")

          self.gate_width = rospy.get_param("gate_width")

          self.pub_mission_display = rospy.Publisher("/mission_display", String, queue_size=1)

     def gate(self, mission_after=None):
          global sm
          if not self.master_planner: 
               sm = smach.StateMachine(outcomes=["success", "failure"])
               sm.open()

          self.pub_mission_display.publish("Gate")
          target_color = rospy.get_param("target_color")

          timer = rospy.Timer(rospy.Duration(self.gate_time_limit), self.mission_timout, oneshot=True)

          target_state_name = mission_after if (self.master_planner and mission_after is not None) else "success"

          smach.StateMachine.add('find_gate', InPlaceSearch(self.control, self.mapping, target_class="Gate", min_objects=1), 
               transitions={'success': 'navigate_gate_go_through', 'failure': 'failure'})
          smach.StateMachine.add('navigate_gate_go_through', NavigateGate(self.control, self.mapping, self.state, target_color, True, self.gate_width), 
               transitions={'success': target_state_name, 'failure':'failure'})

          timer.shutdown()

          if not self.master_planner:  
               res = sm.execute()
               sm.close()
               self.endMission("Finished gate mission. Result {}".format(res))              
          

     def lane_marker(self, second_lane_marker=False, alone=True, mission_after=None):
          global sm
          if alone: 
               sm = smach.StateMachine(outcomes=["success", "failure"])
               sm.open()

          self.pub_mission_display.publish("Lane")
          
          timer = rospy.Timer(rospy.Duration(self.lane_marker_time_limit), self.mission_timout, oneshot=True)
          
          first_state_name = "find_second_lane_marker" if second_lane_marker else "find_lane_marker"
          second_state_name = "navigate_second_lane_marker" if second_lane_marker else "navigate_lane_marker"
          target_state_name = mission_after if (self.master_planner and mission_after is not None) else "success"

          smach.StateMachine.add(first_state_name, BreadthFirstSearch(self.control, self.mapping, target_class="Lane Marker", min_objects=1), 
                    transitions={'success': second_state_name, 'failure': 'failure'})

          smach.StateMachine.add(second_state_name, NavigateLaneMarker(self.control, self.mapping, self.state, origin_class=""), 
                    transitions={"success": target_state_name, 'failure': 'failure'})

          timer.shutdown()

          if not self.master_planner:
               res = sm.execute()
               sm.close()
               self.endMission(f"Finished lane marker. Result {res}")
     
     def buoy(self):
          global sm
          if not self.master_planner: 
               sm = smach.StateMachine(outcomes=["success", "failure"])
               sm.open()

          self.pub_mission_display.publish("Buoy")  

          timer = rospy.Timer(rospy.Duration(self.buoy_time_limit), self.mission_timout, oneshot=True)

          target_state_name = "find_second_lane_marker" if self.master_planner else "success"

          smach.StateMachine.add('find_buoy', InPlaceSearch(self.control, self.mapping, target_class="Buoy", min_objects=1),
                    transitions={'success': 'navigate_buoy', 'failure': 'failure'})
          smach.StateMachine.add('navigate_buoy', NavigateBuoy(self.control, self.mapping, self.state), 
                    transitions={'success': target_state_name, 'failure':'failure'})
          
          timer.shutdown()
          
          if not self.master_planner:     
               res = sm.execute()
               sm.close()
               self.endMission("Finished buoy mission. Result {}".format(res))


     def dropper(self):
          global sm
          if not self.master_planner: 
               sm = smach.StateMachine(outcomes=["success", "failure"])
               sm.open()

          self.pub_mission_display.publish("Dropper")

          timer = rospy.Timer(rospy.Duration(self.dropper_time_limit), self.mission_timout, oneshot=True)

          """
          ADD SOME TRANSITIONS HERE
          """

          timer.shutdown()

          if not self.master_planner:     
               res = sm.execute()
               sm.close()
               self.endMission("Finished dropper mission. Result {}".format(res))


     def octagon(self):
          global sm
          if not self.master_planner: 
               sm = smach.StateMachine(outcomes=["success", "failure"])
               sm.open()

          self.pub_mission_display.publish("Octagon")

          timer = rospy.Timer(rospy.Duration(self.octagon_time_limit), self.mission_timout, oneshot=True)

          smach.StateMachine.add('find_octagon', LinearSearch(self.control, self.mapping, target_class="Octagon Table", min_objects=1), 
               transitions={'success': 'navigate_octagon', 'failure':'failure'})
          smach.StateMachine.add('navigate_octagon', NavigateOctagon(self.control, self.mapping, self.state), 
               transitions={'success': 'success', 'failure':'failure'}) # last task of the competition - no need for if self.master_planner

          timer.shutdown()

          if not self.master_planner: # not necessary since octagon is last task, but clearer code
               res = sm.execute() 
               sm.close()
               self.endMission("Finished buoy mission. Result {}".format(res))

     def torpedo(self):
          global sm
          if not self.master_planner: 
               sm = smach.StateMachine(outcomes=["success", "failure"])
               sm.open()

          self.pub_mission_display.publish("Torpedo")

          timer = rospy.Timer(rospy.Duration(self.torpedo_time_limit), self.mission_timout, oneshot=True)

          """
          ADD SOME TRANSITIONS HERE
          """

          timer.shutdown()

          if not self.master_planner:     
               res = sm.execute()
               sm.close()
               self.endMission("Finished torpedo mission. Result {}".format(res))
     

     def trick(self, trick, num_full_spins, mission_after=None):
          global sm
          if not self.master_planner: 
               sm = smach.StateMachine(outcomes=["success", "failure"])
               sm.open()

          self.pub_mission_display.publish("Trick")    

          timer = rospy.Timer(rospy.Duration(self.tricks_time_limit), self.mission_timout, oneshot=True)

          target_state_name = mission_after if (self.master_planner and mission_after is not None) else "success"

          smach.StateMachine.add('trick', Trick(self.control, trick, num_full_spins), 
               transitions={'success': target_state_name, 'failure':'failure'})
          
          timer.shutdown()

          if not self.master_planner:
               res = sm.execute()
               sm.close()
               self.endMission("Finished trick. Result {}".format(res))


     def mission_timout(self, timer_obj):
          global sm
          self.pub_mission_display.publish("Timeout")
          sm.request_preempt()
          self.endMission("Timeout - mission is taking too long")


     def endMission(self, msg):
          print(msg)
          self.control.freeze_pose()

     
     def endPlanner(self, msg="Shutting down mission planner."):
          self.pub_mission_display.publish("End")
          print(msg)
          self.control.kill()
          
     
     def execute_master_planner(self):
          global sm
          sm = smach.StateMachine(outcomes=["success", "failure"])
          sm.open()
          self.pub_mission_display.publish("Master")

          self.master_planner = True

          print(f"Waiting {self.mission_wait_time} seconds before starting master planner...")
          # rospy.sleep(self.mission_wait_time)
          print("Starting master planner")
          
          self.gate(mission_after="trick")
          self.trick("yaw", 3, mission_after="find_lane_marker")
          self.lane_marker(alone=False, mission_after="find_buoy")
          self.buoy()
          self.lane_marker(second_lane_marker=True, alone=False)

          res = sm.execute()
          sm.close()

          self.master_planner = False

          self.endPlanner("Finished Robosub with result: " + str(res) + "!!!!!!!!!")


if __name__ == '__main__':
     rospy.init_node("competition_planner", log_level=rospy.DEBUG)
     
     try:
          sm = None
          master_planner = False
          missions = Missions(master_planner)
          
          rospy.on_shutdown(missions.endPlanner)
          
          mission_choice = ""
          while mission_choice != "0":
               mission_choice = input("""
     [0] Exit
     [1] Gate
     [2] Lane Marker
     [3] Trick
     [4] Buoy
     [5] Dropper
     [6] Octagon
     [7] Torpedo
     [8] Master Planner 
     Select one of the available missions: """)
               if mission_choice == "0": break
               elif mission_choice == "1": missions.gate()
               elif mission_choice == "2": missions.lane_marker()
               elif mission_choice == "3":

                    trick_choice = input("""
     [0] Roll
     [1] Pitch
     [2] Yaw
     Select one of the available trick: """)
                    trick_times_choice = input("""
     Write the number of times to perform this trick (e.g., 2):  """)
                    trick = ""
                    if trick_choice == "0": trick = "roll"
                    elif trick_choice == "1": trick = "pitch"
                    elif trick_choice == "2": trick = "yaw"
                    else: continue
                    missions.trick(trick, int(trick_times_choice))

               elif mission_choice == "4": missions.buoy()
               elif mission_choice == "5": missions.dropper()
               elif mission_choice == "6": missions.octagon()
               elif mission_choice == "7": missions.torpedo()
               elif mission_choice == "8": missions.execute_master_planner()
               else: print("Invalid input!!!")
     except KeyboardInterrupt:
          #ASSUMING ONE CURRENTLY RUNNING STATE MACHINE AT A TIME (NO THREADS)
          if sm is not None: sm.request_preempt()
     finally:
          missions.endPlanner()
