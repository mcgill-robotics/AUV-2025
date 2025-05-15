#!/usr/bin/env python3
import rospy
from substates.utility.controller import Controller
from substates.utility.functions import countdown
from substates.utility.yaw_controller import YawRamp
import numpy as np

def main():
    rospy.init_node("pooltest")

    # Only need one Controller instantiation
    controls = Controller(rospy.Time(0))
    yaw_ctrl = YawRamp()
    try:
        rospy.loginfo("Executing square pattern movement…")
        print("jdkns")
        yaw_ctrl.run(-90, step_deg=1, tol_deg=1, timeout_s=40)
        # Move forward 0.5 m in the local X direction
        # controls.rotateDeltaEuler([0,0,90])
        print("AAAAAAAAAAAAA")

        # rospy.sleep(10)
        #controls.moveDeltaLocal(-0.5, 0, 0)
        # Then strafe in local Y
        #controls.moveDeltaLocal(0, -0.5, 0)
 
        #controls.moveDeltaLocal(0, 0 , -0.6)
        # rospy.sleep(10)

        # …and so on if you uncomment the rest
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt received. Stopping movements.")
    finally:
        rospy.loginfo("Shutting down controls…")
        controls.kill()

if __name__ == "__main__":
    main()
