#!/usr/bin/env python3

import rospy
import smach
from std_msgs.msg import String

from missions_utils import *
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
from substates.navigate_pinger import *
from substates.octagon_task import *


class Missions:
    def __init__(self):
        self.mapping = ObjectMapper()
        self.state = StateTracker()
        self.control = Controller(rospy.Time(0))

        self.gate_width = rospy.get_param("gate_width")
        self.quali_gate_width = rospy.get_param("quali_gate_width")

        self.nominal_depth = rospy.get_param("nominal_depth")

        self.pinger_number = rospy.get_param("pinger_number") 

        self.pub_mission_display = rospy.Publisher(
            "/mission_display", String, queue_size=1
        )

    def gate(self, first_state_name, count, mission_after):
        global sm

        first_state_name = first_state_name + count
        second_state_name = "navigate_gate_go_through" + count
        target_state_name = mission_after if mission_after is not None else "success"

        smach.StateMachine.add(
            first_state_name,
            InPlaceSearch(
                self.control, self.mapping, target_class="Gate", min_objects=1
            ),
            transitions={"success": second_state_name,
                         "timeout": target_state_name, 
                         "failure": "failure"},
        )
        smach.StateMachine.add(
            second_state_name,
            NavigateGate(
                control=self.control,
                mapping=self.mapping,
                state=self.state,
                goThrough=True
            ),
            transitions={"success": target_state_name, 
                         "timeout": target_state_name,
                         "failure": "failure"},
        )

    def lane_marker(self, first_state_name, count, mission_after):
        global sm

        first_state_name = first_state_name + count
        second_state_name = "navigate_lane_marker" + count
        target_state_name = mission_after if mission_after is not None else "success"

        timer = rospy.Timer(
            rospy.Duration(self.lane_marker_time_limit), 
            lambda event: self.mission_timeout("Lane Marker", self.lane_marker_time_limit, event), 
            oneshot=True
        )

        smach.StateMachine.add(
            first_state_name,
            BreadthFirstSearch(
                self.control,
                self.mapping,
                target_class="Lane Marker",
                min_objects=int(count),
            ),
            transitions={"success": second_state_name, "failure": "failure"},
        )
        smach.StateMachine.add(
            second_state_name,
            NavigateLaneMarker(self.control, self.mapping, self.state, origin_class=""),
            transitions={"success": target_state_name, "failure": "failure"},
        )

        timer.shutdown()

    def pinger(self, first_state_name, count, mission_after):
        global sm

        first_state_name = first_state_name + count
        target_state_name = mission_after if mission_after is not None else "success"

        timer = rospy.Timer(
            rospy.Duration(self.pinger_time_limit), 
            lambda event: self.mission_timeout("Pinger", self.pinger_time_limit, event), 
            oneshot=True
        )

        smach.StateMachine.add(
            first_state_name,
            NavigatePinger(
                control=self.control, 
                state=self.state, 
                mapping=self.mapping, 
                pinger_number=self.pinger_number
            ),
            transitions={"success": target_state_name, "failure": "failure"}
        )

        self.pinger_number += 1

    def buoy(self, first_state_name, count, mission_after):
        global sm
        
        first_state_name = first_state_name + count
        second_state_name = "navigate_buoy" + count
        target_state_name = mission_after if mission_after is not None else "success"

        timer = rospy.Timer(
            rospy.Duration(self.buoy_time_limit), 
            lambda event: self.mission_timeout("Buoy", self.buoy_time_limit, event), 
            oneshot=True
        )

        smach.StateMachine.add(
            first_state_name,
            InPlaceSearch(
                self.control, self.mapping, target_class="Buoy", min_objects=1
            ),
            transitions={"success": second_state_name, "failure": "failure"},
        )
        smach.StateMachine.add(
            second_state_name,
            NavigateBuoy(self.control, self.mapping, self.state),
            transitions={"success": target_state_name, "failure": "failure"},
        )

        timer.shutdown()

    def dropper(self, first_state_name, count, mission_after):
        global sm

        timer = rospy.Timer(
            rospy.Duration(self.dropper_time_limit), 
            lambda event: self.mission_timeout("Dropper", self.dropper_time_limit, event), 
            oneshot=True
        )

        """
        ADD SOME TRANSITIONS HERE
        """

        timer.shutdown()

    def octagon(self, first_state_name, count, mission_after):
        global sm

        first_state_name = first_state_name + count
        second_state_name = "navigate_octagon" + count
        target_state_name = mission_after if mission_after is not None else "success"

        timer = rospy.Timer(
            rospy.Duration(self.octagon_time_limit), 
            lambda event: self.mission_timeout("Octagon", self.octagon_time_limit, event), 
            oneshot=True
        )

        smach.StateMachine.add(
            first_state_name,
            LinearSearch(
                self.control, self.mapping, target_class="Octagon Table", min_objects=1
            ),
            transitions={"success": second_state_name, "failure": "failure"},
        )
        smach.StateMachine.add(
            second_state_name,
            NavigateOctagon(self.control, self.mapping, self.state),
            transitions={"success": target_state_name, "failure": "failure"},
        )

        timer.shutdown()

    def torpedo(self, first_state_name, count, mission_after):
        global sm

        timer = rospy.Timer(
            rospy.Duration(self.torpedo_time_limit), 
            lambda event: self.mission_timeout("Torpedo", self.torpedo_time_limit, event), 
            oneshot=True
        )

        """
          ADD SOME TRANSITIONS HERE
          """

        timer.shutdown()

    def trick(self, first_state_name, count, mission_after):
        global sm

        first_state_name = first_state_name + count
        target_state_name = mission_after if mission_after is not None else "success"

        timer = rospy.Timer(
            rospy.Duration(self.tricks_time_limit), 
            lambda event: self.mission_timeout("Torpedo", self.tricks_time_limit, event), 
            oneshot=True
        )

        smach.StateMachine.add(
            first_state_name,
            Trick(self.control, "roll", 2),
            transitions={"success": target_state_name, "failure": "failure"},
        )

        timer.shutdown()

    def bins(self, first_state_name, count, mission_after):
        global sm

        timer = rospy.Timer(
            rospy.Duration(self.bins_time_limit), 
            lambda event: self.mission_timeout("Bins", self.bins_time_limit, event), 
            oneshot=True
        )

        """
        ADD SOME TRANSITIONS HERE
        """

        timer.shutdown()

    def quali(self, first_state_name, count, mission_after):
        print("Submerging")
        self.controls.move((None, None, self.nominal_depth))

        print("Navigating gate")
        gateNav = NavigateGate(
            control=self.controls,
            mapping=self.mapping,
            state=self.state,
            goThrough=False
        )
        gateNav.execute(None)

        print("Moving to right side of gate")
        self.controls.moveDeltaLocal((0, -self.quali_gate_width / 4, 0))

        print("Moving through gate")
        self.controls.moveDeltaLocal((14, 0, 0))

        print("Rotating around pole")
        self.controls.rotateDeltaEuler((0, 0, 90))

        self.controls.moveDeltaLocal((self.quali_gate_width / 2, 0, 0))

        self.controls.rotateDeltaEuler((0, 0, 90))

        print("Returning to origin")
        self.controls.moveDeltaLocal((17, 0, 0))

    def endPlanner(self, msg="Shutting down mission planner."):
        self.pub_mission_display.publish("End")
        print(msg)
        self.control.kill()

if __name__ == "__main__":
    rospy.init_node("competition_planner")

    sm = None
    missions = Missions()

    rospy.on_shutdown(missions.endPlanner)

    mission_options = [
        # (mission name, mission function, default first state name, num of times selected by user).
        ["Gate", missions.gate, "find_gate", 0],
        ["Lane Marker", missions.lane_marker, "find_lane_marker", 0],
        ["Trick", missions.trick, "trick", 0],
        ["Buoy", missions.buoy, "find_buoy", 0],
        ["Octagon", missions.octagon, "find_octagon", 0],
        ["Pinger", missions.pinger, "navigate_pinger", 0],
        ["Torpedo", missions.torpedo, "", 0],
        ["Dropper", missions.dropper, "", 0],
        ["Bins", missions.bins, "", 0],
        ["Quali", missions.quali, None, 0],
        ["Competition", None, None, 0],
    ]

    try:
        done = False
        while not done:           
            missions_selected = get_user_missions_selected(mission_options)

            sm = smach.StateMachine(outcomes=["success", "failure"])
            sm.open()

            for i in range(len(missions_selected)):
                current_mission_name, current_mission_count, mission_after = get_state_params(mission_options, missions_selected, i)

                # Add task to state machine.
                mission_options[missions_selected[i]][1](
                    first_state_name=current_mission_name,
                    count=str(current_mission_count),
                    mission_after=mission_after,
                )

            # Execute state machine.
            res = sm.execute()
            sm.close()
            missions.endPlanner(
                "Finished planner with result: " + str(res) + "!!!!!!!!!"
            )

    except KeyboardInterrupt:
        # ASSUMING ONE CURRENTLY RUNNING STATE MACHINE AT A TIME (NO THREADS).
        if sm is not None:
            sm.request_preempt()
    finally:
        missions.endPlanner()

    rospy.spin()