#!/usr/bin/env python3
import rospy
from substates.utility.controller import Controller
from substates.utility.functions import countdown

def main():
    rospy.init_node("pooltest")

    controls = Controller(rospy.Time(0))

    try:
        #controls.rotateDeltaEuler([0.5,0,0])
        # Reset orientation to default
        #rospy.loginfo("Moving down by 0.5 meters...")
        # controls.moveDelta([0, 0, -0.5])
        #rospy.sleep(10)  

        # Move in a square pattern locally
        rospy.loginfo("Executing square pattern movement...")
        controls.moveDelta([0, 0, -0.5])
        rospy.sleep(10)

        # controls.moveDeltaLocal([0, 0.5, 0])
        # rospy.sleep(10)

        # controls.moveDeltaLocal([-0.5, 0, 0])
        # rospy.sleep(10)

        # controls.moveDeltaLocal([0, -0.5, 0])
        # rospy.sleep(10)

    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt received. Stopping movements.")
    finally:
        rospy.loginfo("Shutting down controls...")
        controls.kill()  # Ensure the robot is stopped

if __name__ == "__main__":
    main()
