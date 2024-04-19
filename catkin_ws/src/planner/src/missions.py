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

        self.gate_width = rospy.get_param("gate_width")
        self.quali_gate_width = rospy.get_param("quali_gate_width")

        self.target_color = rospy.get_param("target_color")
        self.nominal_depth = rospy.get_param("nominal_depth")

        self.pub_mission_display = rospy.Publisher(
            "/mission_display", String, queue_size=1
        )

    def gate(self, first_state_name, count, mission_after):
        global sm

        timer = rospy.Timer(
            rospy.Duration(self.gate_time_limit), self.mission_timeout, oneshot=True
        )

        first_state_name = first_state_name + count
        second_state_name = "navigate_gate_go_through" + count
        target_state_name = mission_after if mission_after is not None else "success"

        smach.StateMachine.add(
            first_state_name,
            InPlaceSearch(
                self.control, self.mapping, target_class="Gate", min_objects=1
            ),
            transitions={"success": second_state_name, "failure": "failure"},
        )
        smach.StateMachine.add(
            second_state_name,
            NavigateGate(
                self.control,
                self.mapping,
                self.state,
                self.target_color,
                True,
                self.gate_width,
            ),
            transitions={"success": target_state_name, "failure": "failure"},
        )

        timer.shutdown()

    def lane_marker(self, first_state_name, count, mission_after):
        global sm

        timer = rospy.Timer(
            rospy.Duration(self.lane_marker_time_limit),
            self.mission_timeout,
            oneshot=True,
        )

        first_state_name = first_state_name + count
        second_state_name = "navigate_lane_marker" + count
        target_state_name = mission_after if mission_after is not None else "success"

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

    def buoy(self, first_state_name, count, mission_after):
        global sm

        timer = rospy.Timer(
            rospy.Duration(self.buoy_time_limit), 
            self.mission_timeout,
            oneshot=True
        )

        first_state_name = first_state_name + count
        second_state_name = "navigate_buoy" + count
        target_state_name = mission_after if mission_after is not None else "success"

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
            rospy.Duration(self.dropper_time_limit), self.mission_timeout, oneshot=True
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
            rospy.Duration(self.octagon_time_limit), self.mission_timeout, oneshot=True
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
            rospy.Duration(self.torpedo_time_limit), self.mission_timeout, oneshot=True
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
            self.mission_timeout, 
            oneshot=True,
        )

        smach.StateMachine.add(
            first_state_name,
            Trick(self.control, "roll", 2),
            transitions={"success": target_state_name, "failure": "failure"},
        )

        timer.shutdown()

    def bins(self, first_state_name, count, mission_after):
        pass

    def quali(self, first_state_name, count, mission_after):
        print("Submerging")
        self.controls.move((None, None, self.nominal_depth))

        print("Navigating gate")
        gateNav = NavigateGate(
            self.controls,
            self.state,
            self.mapping,
            self.target_color,
            goThrough=False,
            gate_width=self.quali_gate_width,
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

    def mission_timeout(self, timer_obj):
        global sm
        self.pub_mission_display.publish("Timeout")
        sm.request_preempt()
        print("Timeout - mission is taking too long")

    def endPlanner(self, msg="Shutting down mission planner."):
        self.pub_mission_display.publish("End")
        print(msg)
        self.control.kill()


if __name__ == "__main__":
    rospy.init_node("competition_planner", log_level=rospy.DEBUG)

    sm = None
    missions = Missions()

    rospy.on_shutdown(missions.endPlanner)

    mission_options = [
        # (mission option, corresponding function, default first state name, num of times selected by user)
        ["Gate", missions.gate, "find_gate", 0],
        ["Lane Marker", missions.lane_marker, "find_lane_marker", 0],
        ["Trick", missions.trick, "trick", 0],
        ["Buoy", missions.buoy, "find_buoy", 0],
        ["Octagon", missions.octagon, "find_octagon", 0],
        ["Torpedo", missions.torpedo, "", 0],
        ["Dropper", missions.dropper, "", 0],
        ["Bins", missions.bins, "", 0],
        ["Quali", missions.quali, None, 0],
        ["Master Planner", None, None, 0],
    ]

    master_planner_index = [
        mission_option[0] == "Master Planner" for mission_option in mission_options
    ].index(True)

    try:
        done = False
        while not done:
            # Print all mission options
            for i in range(mission_options):
                print("[{}] ".format(i) + mission_options[i][0])

            # Check if all selected options are valid
            invalid_answer = True
            while invalid_answer:
                missions_selected = list(map(int, input("Select missions (separated by comma [1,2,3]): ").split(",")))
                if len(missions_selected) > 0 and all(0 <= selected < len(mission_options) for selected in missions_selected):
                    invalid_answer = False
                else:
                    print("Invalid answer!!!")

            sm = smach.StateMachine(outcomes=["success", "failure"])
            sm.open()

            # If master planner - write default values for missions selected and tricks - ignore user
            if master_planner_index in missions_selected:
                # gate -> tricks (yaw x 3) -> lane marker -> buoy -> lane marker -> ... @TODO
                missions_selected = [1, 3, 2, 4, 2]
                trick_selected = ["yaw"]
                num_spins_trick_selected = [3]
                print(
                    "Master planner mission found in input, master planner will be the only mission to run."
                )

            len_mission_selected = len(missions_selected)
            for i in range(len_mission_selected):
                # increment specific mission count
                mission_options[missions_selected[i]][3] += 1

                current_mission_name = mission_options[missions_selected[i]][2]
                current_mission_count = mission_options[missions_selected[i]][3]
                # if last mission -> no mission after; else -> get state name of next mission
                if i == len_mission_selected - 1:
                    mission_after = None
                else:
                    next_mission_selected = missions_selected[i + 1]
                    mission_after = mission_options[next_mission_selected][2] + str(mission_options[next_mission_selected][3] + 1)

                # If current mission is trick
                mission_options[missions_selected[i]][1](
                    first_state_name=current_mission_name,
                    count=str(current_mission_count),
                    mission_after=mission_after,
                )

            res = sm.execute()
            sm.close()
            missions.endPlanner(
                "Finished planner with result: " + str(res) + "!!!!!!!!!"
            )

    except KeyboardInterrupt:
        # ASSUMING ONE CURRENTLY RUNNING STATE MACHINE AT A TIME (NO THREADS)
        if sm is not None:
            sm.request_preempt()
    finally:
        missions.endPlanner()
