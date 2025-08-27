#!/usr/bin/env python3
import rospy
from substates.utility.controller import Controller
from substates.utility.functions import countdown


pub_mission_display = rospy.Publisher("/mission_display", String, queue_size=1)

def update_display(status):
    pub_mission_display.publish(status)
    print(status)

def main():
    rospy.init_node("semis")

    timer = 5
    update_display(f"{timer} COUNTDOWN",)
    rospy.loginfo(f"Starting {timer} countdown")
    countdown(timer)

    controls = Controller(rospy.Time(0))
    try:
        #moveDeltaLocal(self, delta_x, delta_y, delta_z, tolerance=0.05, timeout=30)
        #rotate(self, x: float, y: float, z: float, timeout: float = 20, tol_degrees: float = 1.0) 
        #positive = CCW rotation

        #SUBMERGE
        controls.moveDeltaLocal(0,0, -1)

        #TASK: COIN FLIP 300
        #Tails = 180
        #Heads = 90
        # controls.rotate(0,0, 180)   #TODO

        #TASK: GATE 250 + SLALOM
        controls.moveDeltaLocal(19, 0, 0, tolerance = 0.1, timeout = 60)
        rospy.sleep(2)
        
        #TASK: OCTAGON
        controls.moveDeltaLocal(0,0,1, tolerance = 0.1, timeout = 60)
        rospy.sleep(2)
        controls.moveDeltaLocal(0, 0, -1, tolerance = 0.1, timeout = 60)

        #TASK: RETURN HOME
        controls.moveDeltaLocal(-16, 0, 0, tolerance = 0.1, timeout = 60)

        rospy.sleep(2)

        #TASK: TRICKS YAW 800
        i=0
        num_rotations = 8
        while i < num_rotations: 
            controls.rotate(0, 0, 90)
            i+=1

        rospy.sleep(3)

    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt received. Stopping movements.")
    finally:
        rospy.loginfo("Shutting down controls…")
        controls.kill()

if __name__ == "__main__":
    main()
