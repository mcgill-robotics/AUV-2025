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


     def gate(self, first_state_name, count, mission_after):
          global sm
          
          target_color = rospy.get_param("target_color")

          timer = rospy.Timer(rospy.Duration(self.gate_time_limit), self.mission_timeout, oneshot=True)

          first_state_name = first_state_name + count
          second_state_name = "navigate_gate_go_through" + count
          target_state_name = mission_after if mission_after is not None else "success"

          smach.StateMachine.add(first_state_name, InPlaceSearch(self.control, self.mapping, target_class="Gate", min_objects=1), 
               transitions={"success": second_state_name, "failure": "failure"})
          smach.StateMachine.add(second_state_name, NavigateGate(self.control, self.mapping, self.state, target_color, True, self.gate_width), 
               transitions={"success": target_state_name, "failure":"failure"})

          timer.shutdown()           
          

     def lane_marker(self, first_state_name, count, mission_after):
          global sm
          
          timer = rospy.Timer(rospy.Duration(self.lane_marker_time_limit), self.mission_timeout, oneshot=True)
          
          first_state_name = first_state_name + count
          second_state_name = "navigate_lane_marker" + count
          target_state_name = mission_after if mission_after is not None else "success"

          smach.StateMachine.add(first_state_name, BreadthFirstSearch(self.control, self.mapping, target_class="Lane Marker", min_objects=1), 
                    transitions={"success": second_state_name, "failure": "failure"})
          smach.StateMachine.add(second_state_name, NavigateLaneMarker(self.control, self.mapping, self.state, origin_class=""), 
                    transitions={"success": target_state_name, "failure": "failure"})

          timer.shutdown()

     
     def buoy(self, first_state_name, count, mission_after):
          global sm

          timer = rospy.Timer(rospy.Duration(self.buoy_time_limit), self.mission_timeout, oneshot=True)

          first_state_name = first_state_name + count
          second_state_name = "navigate_buoy" + count
          target_state_name = mission_after if mission_after is not None else "success"

          smach.StateMachine.add(first_state_name, InPlaceSearch(self.control, self.mapping, target_class="Buoy", min_objects=1),
                    transitions={"success": second_state_name, "failure": "failure"})
          smach.StateMachine.add(second_state_name, NavigateBuoy(self.control, self.mapping, self.state), 
                    transitions={"success": target_state_name, "failure":"failure"})
          
          timer.shutdown()


     def dropper(self, first_state_name, count, mission_after):          
          global sm

          timer = rospy.Timer(rospy.Duration(self.dropper_time_limit), self.mission_timeout, oneshot=True)

          """
          ADD SOME TRANSITIONS HERE
          """

          timer.shutdown()


     def octagon(self, first_state_name, count, mission_after):
          global sm

          first_state_name = first_state_name + count
          second_state_name = "navigate_octagon" + count
          target_state_name = mission_after if mission_after is not None else "success"

          timer = rospy.Timer(rospy.Duration(self.octagon_time_limit), self.mission_timeout, oneshot=True)

          smach.StateMachine.add(first_state_name, LinearSearch(self.control, self.mapping, target_class="Octagon Table", min_objects=1), 
               transitions={"success": second_state_name, "failure":"failure"})
          smach.StateMachine.add(second_state_name, NavigateOctagon(self.control, self.mapping, self.state), 
               transitions={"success": target_state_name, "failure":"failure"}) # last task of the competition - no need for if self.multiple_missions

          timer.shutdown()


     def torpedo(self, first_state_name, count, mission_after):
          global sm

          timer = rospy.Timer(rospy.Duration(self.torpedo_time_limit), self.mission_timeout, oneshot=True)

          """
          ADD SOME TRANSITIONS HERE
          """

          timer.shutdown()
     

     def trick(self, first_state_name, count, mission_after, trick, num_full_spins):
          global sm                      

          first_state_name = first_state_name + count
          target_state_name = mission_after if mission_after is not None else "success"

          timer = rospy.Timer(rospy.Duration(self.tricks_time_limit), self.mission_timeout, oneshot=True)

          smach.StateMachine.add(first_state_name, Trick(self.control, trick, num_full_spins), 
               transitions={"success": target_state_name, "failure":"failure"})
          
          timer.shutdown()


     def bins(self, first_state_name, count, mission_after):
          pass

     
     def quali(self, first_state_name, count, mission_after):
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


     def mission_timeout(self, timer_obj):
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

     sm = None
     missions = Missions()
     
     rospy.on_shutdown(missions.endPlanner)

     mission_options = [
          # (mission option, corresponding function, default first state name, num of times selected by user)
          ["[0] Exit", None, None, 0],
          ["[1] Gate", missions.gate, "find_gate", 0],
          ["[2] Lane Marker", missions.lane_marker, "find_lane_marker", 0],
          ["[3] Trick", missions.trick, "trick", 0],
          ["[4] Buoy", missions.buoy, "find_buoy", 0],
          ["[5] Octagon", missions.octagon, "find_octagon", 0],
          ["[6] Torpedo", missions.torpedo, "", 0],
          ["[7] Dropper", missions.dropper, "", 0],
          ["[8] Bins", missions.bins, "", 0],
          ["[9] Quali", missions.quali, None, 0],
          ["[10] Master Planner", None, None, 0]
     ]

     trick_options = [
          ["[0] Roll", "roll"],
          ["[1] Pitch", "pitch"],
          ["[2] Yaw", "yaw"]
     ]
     
     try:
          done = False
          while not done:
               # Print all mission options
               for option in mission_options:
                    print(option[0])

               # Check if all selected options are valid
               invalid_answer = True
               while invalid_answer:
                    missions_selected = list(map(int, input("Select missions (separated by comma [1,2,3]): ").split(",")))
                    if len(missions_selected) > 0 and all(0 <= selected < len(mission_options) for selected in missions_selected):
                         invalid_answer = False
                    else:
                         print("Invalid answer!!!")
               
               is_master_planner = False
               if 10 in missions_selected:
                    is_master_planner = True

               sm = smach.StateMachine(outcomes=["success", "failure"])
               sm.open()  
               
               # Exit
               if 0 in missions_selected: 
                    done = True
               else:
                    # If master planner - write default values for missions selected and tricls - ignore user
                    if is_master_planner:
                         # gate -> tricks (yaw x 3) -> lane marker -> buoy -> lane marker -> ...
                         missions_selected = [1, 3, 2, 4, 2]
                         trick_selected = ["yaw"]
                         num_spins_trick_selected = [3]
                         is_master_planner = False
                    else:
                         # Check if trick is in selected missions -> if yes, ask which tricks and how many times
                         total_num_tricks = missions_selected.count(3)
                         trick_selected = []
                         num_spins_trick_selected = []
                         if total_num_tricks > 0:
                              # Print all trick options
                              for trick in trick_options:
                                   print(trick)
                              # Check if the number of tricks are the same across all answers
                              invalid_answer = True
                              while invalid_answer:
                                   trick_input = list(map(int, input("Select tricks (separated by comma [0,1,1]): ").split(",")))
                                   trick_selected = [trick_options[i][1] for i in trick_input]
                                   num_spins_trick_selected = list(map(int, input("Select number of times to perform each trick (separated by comma): ").split(",")))
                                   if len(trick_selected) != total_num_tricks or len(num_spins_trick_selected) != total_num_tricks:
                                        print("Invalid number of tricks!!!")
                                   else: 
                                        invalid_answer = False

                    len_mission_selected = len(missions_selected)
                    # If more than one mission
                    if len_mission_selected > 1:
                         missions.multiple_missions = True

                    trick_i = 0
                    for i in range(len_mission_selected):
                         # increment specific mission count
                         mission_options[missions_selected[i]][3] += 1

                         current_mission_name = mission_options[missions_selected[i]][2]
                         current_mission_cout = mission_options[missions_selected[i]][3]
                         # if last mission -> no mission after; else -> get state name of next mission
                         if i == len_mission_selected - 1:
                              mission_after = None
                         else:
                              next_mission_selected = missions_selected[i + 1]
                              mission_after = mission_options[next_mission_selected][2] + str(mission_options[next_mission_selected][3] + 1)
                         
                         # If current mission is trick
                         if missions_selected[i] == 3:
                              current_trick = trick_selected[trick_i] # trick name (e.g., "roll")
                              times_current_trick = num_spins_trick_selected[trick_i] # num of times to perform trick (e.g. roll twice)
                              mission_options[missions_selected[i]][1](
                                   first_state_name=current_mission_name, 
                                   count=str(current_mission_cout),
                                   mission_after=mission_after,
                                   trick=current_trick, 
                                   num_full_spins=times_current_trick
                              ) 
                              trick_i += 1
                         else:
                              mission_options[missions_selected[i]][1](
                                   first_state_name=current_mission_name,
                                   count=str(current_mission_cout),
                                   mission_after=mission_after
                              )  

               res = sm.execute()
               sm.close()
               missions.endPlanner("Finished planner with result: " + str(res) + "!!!!!!!!!")
                    
                         
     except KeyboardInterrupt:
          #ASSUMING ONE CURRENTLY RUNNING STATE MACHINE AT A TIME (NO THREADS)
          if sm is not None: sm.request_preempt()
     finally:
          missions.endPlanner()
