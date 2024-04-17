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
     def __init__(self):
          self.multiple_missions = False
          
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
          self.quali_gate_width = rospy.get_param("quali_gate_width")

          self.target_color = rospy.get_param("target_color")
          self.nominal_depth = rospy.get_param("nominal_depth")

          self.pub_mission_display = rospy.Publisher("/mission_display", String, queue_size=1)

          # Name of first state for each mission
          self.gate_first_state = "find_gate"
          self.lane_first_time_first_state = "find_lane_marker"
          self.lane_second_time_first_state = "find_second_lane_marker"
          self.buoy_first_state = "find_buoy"
          self.octagon_first_state = "find_octagon"
          self.trick_first_state = "trick"
          self.dropper_first_state = ""
          self.torpedo_first_state = ""



     def gate(self, mission_after=None):
          global sm
          if not self.multiple_missions: 
               sm = smach.StateMachine(outcomes=["success", "failure"])
               sm.open()
          
          target_color = rospy.get_param("target_color")

          timer = rospy.Timer(rospy.Duration(self.gate_time_limit), self.mission_timout, oneshot=True)

          target_state_name = mission_after if (self.multiple_missions and mission_after is not None) else "success"

          smach.StateMachine.add(self.gate_first_state, InPlaceSearch(self.control, self.mapping, target_class="Gate", min_objects=1), 
               transitions={'success': 'navigate_gate_go_through', 'failure': 'failure'})
          smach.StateMachine.add('navigate_gate_go_through', NavigateGate(self.control, self.mapping, self.state, target_color, True, self.gate_width), 
               transitions={'success': target_state_name, 'failure':'failure'})

          timer.shutdown()

          if not self.multiple_missions:  
               res = sm.execute()
               sm.close()
               self.endMission("Finished gate mission. Result {}".format(res))              
          

     def lane_marker(self, second_lane_marker=False, alone=True, mission_after=None):
          global sm
          if alone: 
               sm = smach.StateMachine(outcomes=["success", "failure"])
               sm.open()          
          
          timer = rospy.Timer(rospy.Duration(self.lane_marker_time_limit), self.mission_timout, oneshot=True)
          
          first_state_name = self.lane_second_time_first_state if second_lane_marker else self.lane_first_time_first_state
          second_state_name = "navigate_second_lane_marker" if second_lane_marker else "navigate_lane_marker"
          target_state_name = mission_after if (self.multiple_missions and mission_after is not None) else "success"

          smach.StateMachine.add(first_state_name, BreadthFirstSearch(self.control, self.mapping, target_class="Lane Marker", min_objects=1), 
                    transitions={'success': second_state_name, 'failure': 'failure'})

          smach.StateMachine.add(second_state_name, NavigateLaneMarker(self.control, self.mapping, self.state, origin_class=""), 
                    transitions={"success": target_state_name, 'failure': 'failure'})

          timer.shutdown()

          if not self.multiple_missions:
               res = sm.execute()
               sm.close()
               self.endMission(f"Finished lane marker. Result {res}")
     
     def buoy(self, mission_after=None):
          global sm
          if not self.multiple_missions: 
               sm = smach.StateMachine(outcomes=["success", "failure"])
               sm.open()

          timer = rospy.Timer(rospy.Duration(self.buoy_time_limit), self.mission_timout, oneshot=True)

          target_state_name = "find_second_lane_marker" if self.multiple_missions else "success"

          smach.StateMachine.add(self.buoy_first_state, InPlaceSearch(self.control, self.mapping, target_class="Buoy", min_objects=1),
                    transitions={'success': 'navigate_buoy', 'failure': 'failure'})
          smach.StateMachine.add('navigate_buoy', NavigateBuoy(self.control, self.mapping, self.state), 
                    transitions={'success': target_state_name, 'failure':'failure'})
          
          timer.shutdown()
          
          if not self.multiple_missions:     
               res = sm.execute()
               sm.close()
               self.endMission("Finished buoy mission. Result {}".format(res))


     def dropper(self, mission_after=None):          
          # self.pub_mission_display.publish("Dropper")

          global sm
          if not self.multiple_missions: 
               sm = smach.StateMachine(outcomes=["success", "failure"])
               sm.open()

          timer = rospy.Timer(rospy.Duration(self.dropper_time_limit), self.mission_timout, oneshot=True)

          """
          ADD SOME TRANSITIONS HERE
          """

          timer.shutdown()

          if not self.multiple_missions:     
               res = sm.execute()
               sm.close()
               self.endMission("Finished dropper mission. Result {}".format(res))


     def octagon(self, mission_after=None):
          global sm
          if not self.multiple_missions: 
               sm = smach.StateMachine(outcomes=["success", "failure"])
               sm.open()

          timer = rospy.Timer(rospy.Duration(self.octagon_time_limit), self.mission_timout, oneshot=True)

          smach.StateMachine.add(self.octagon_first_state, LinearSearch(self.control, self.mapping, target_class="Octagon Table", min_objects=1), 
               transitions={'success': 'navigate_octagon', 'failure':'failure'})
          smach.StateMachine.add('navigate_octagon', NavigateOctagon(self.control, self.mapping, self.state), 
               transitions={'success': 'success', 'failure':'failure'}) # last task of the competition - no need for if self.multiple_missions

          timer.shutdown()

          if not self.multiple_missions: # not necessary since octagon is last task, but clearer code
               res = sm.execute() 
               sm.close()
               self.endMission("Finished buoy mission. Result {}".format(res))

     def torpedo(self, mission_after=None):
          # self.pub_mission_display.publish("Torpedo")
          global sm
          if not self.multiple_missions: 
               sm = smach.StateMachine(outcomes=["success", "failure"])
               sm.open()

          timer = rospy.Timer(rospy.Duration(self.torpedo_time_limit), self.mission_timout, oneshot=True)

          """
          ADD SOME TRANSITIONS HERE
          """

          timer.shutdown()

          if not self.multiple_missions:     
               res = sm.execute()
               sm.close()
               self.endMission("Finished torpedo mission. Result {}".format(res))
     

     def trick(self, trick, num_full_spins, mission_after=None):
          global sm
          if not self.multiple_missions: 
               sm = smach.StateMachine(outcomes=["success", "failure"])
               sm.open()            

          timer = rospy.Timer(rospy.Duration(self.tricks_time_limit), self.mission_timout, oneshot=True)

          target_state_name = mission_after if (self.multiple_missions and mission_after is not None) else "success"

          smach.StateMachine.add(self.trick_first_state, Trick(self.control, trick, num_full_spins), 
               transitions={'success': target_state_name, 'failure':'failure'})
          
          timer.shutdown()

          if not self.multiple_missions:
               res = sm.execute()
               sm.close()
               self.endMission("Finished trick. Result {}".format(res))

     
     def quali(self):
          print("Submerging")
          self.controls.move((None, None, self.nominal_depth))

          print("Navigating gate")
          gateNav = NavigateGate(self.controls, self.state, self.mapping, self.target_color, goThrough=False, gate_width=self.quali_gate_width)
          gateNav.execute(None)

          print("Moving to right side of gate")
          self.controls.moveDeltaLocal((0, -self.quali_gate_width/4, 0))

          print("Moving through gate")
          self.controls.moveDeltaLocal((14, 0, 0))

          print("Rotating around pole")
          self.controls.rotateDeltaEuler((0, 0, 90))

          self.controls.moveDeltaLocal((self.quali_gate_width / 2, 0, 0))

          self.controls.rotateDeltaEuler((0, 0, 90))

          print("Returning to origin")
          self.controls.moveDeltaLocal((17, 0, 0))

     
     def execute_master_planner(self):
          self.pub_mission_display.publish("Master")

          global sm
          sm = smach.StateMachine(outcomes=["success", "failure"])
          sm.open()

          self.multiple_missions = True

          print(f"Waiting {self.mission_wait_time} seconds before starting master planner...")
          rospy.sleep(self.mission_wait_time)
          print("Starting master planner")
          
          self.gate(mission_after=self.trick_first_state)
          self.trick("yaw", 3, mission_after=self.lane_first_time_first_state)
          self.lane_marker(alone=False, mission_after=self.buoy_first_state)
          self.buoy(mission_after=self.lane_second_time_first_state)
          self.lane_marker(second_lane_marker=True, alone=False)

          res = sm.execute()
          sm.close()

          self.multiple_missions = False

          self.endPlanner("Finished Robosub with result: " + str(res) + "!!!!!!!!!")

     
     def custom(self, custom_choices):
          self.pub_mission_display.publish("Custom")

          global sm
          sm = smach.StateMachine(outcomes=["success", "failure"])
          sm.open()

          self.multiple_missions = True

          first_state_sequence = []
          lane_first_time = True

          for i in range(len(custom_choices)):
               if custom_choices[i] == "1":   first_state_sequence.append(self.gate_first_state)
               elif custom_choices[i] == "2": 
                    if lane_first_time:       first_state_sequence.append(self.lane_first_time_first_state)
                    else:                     first_state_sequence.append(self.lane_second_time_first_state)
                    lane_first_time = False
               elif custom_choices[i] == "3": first_state_sequence.append(self.trick_first_state)
               elif custom_choices[i] == "4": first_state_sequence.append(self.buoy_first_state)
               elif custom_choices[i] == "5": first_state_sequence.append(self.dropper_first_state)
               elif custom_choices[i] == "6": first_state_sequence.append(self.octagon_first_state)
               elif custom_choices[i] == "7": first_state_sequence.append(self.torpedo_first_state)
               else:
                    print("Invalid input!!!")
                    sm.close()
                    return

          len_first_state_sequence = len(first_state_sequence)

          for i in range(len_first_state_sequence):
               # Check if last mission
               if i == len_first_state_sequence - 1:
                    mission_after = None
               else:
                    mission_after = first_state_sequence[i + 1]
               if first_state_sequence[i] == self.gate_first_state:               self.gate(mission_after=mission_after)
               elif first_state_sequence[i] == self.lane_first_time_first_state:  self.lane_marker(alone=False, mission_after=mission_after)
               elif first_state_sequence[i] == self.lane_second_time_first_state: self.lane_marker(alone=False, mission_after=mission_after, second_lane_marker=True)
               elif first_state_sequence[i] == self.trick_first_state:            self.trick("yaw", 3, mission_after=mission_after)
               elif first_state_sequence[i] == self.buoy_first_state:             self.buoy(mission_after=mission_after)
               elif first_state_sequence[i] == self.dropper_first_state:          self.dropper(mission_after=mission_after)
               elif first_state_sequence[i] == self.octagon_first_state:          self.octagon(mission_after=mission_after)
               elif first_state_sequence[i] == self.torpedo_first_state:          self.torpedo(mission_after=mission_after)
          
          sm.execute()
          sm.close()
          self.multiple_missions = False

          self.endPlanner("Finished custom mission with result: " + str(res))



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
          


if __name__ == '__main__':
     rospy.init_node("competition_planner", log_level=rospy.DEBUG)
     
     try:
          sm = None
          missions = Missions()
          
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
     [9] Quali
     [10] Custom
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
               elif mission_choice == "9": missions.quali()
               elif mission_choice == "10": 
                    custom_sequence_choice = input("""
     Select the missions from 1-7 (e.g., 123): """) 
                    missions.custom(custom_sequence_choice)
               else: print("Invalid input!!!")
     except KeyboardInterrupt:
          #ASSUMING ONE CURRENTLY RUNNING STATE MACHINE AT A TIME (NO THREADS)
          if sm is not None: sm.request_preempt()
     finally:
          missions.endPlanner()
