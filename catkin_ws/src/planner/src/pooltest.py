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
        # negative is CCW, thus positive is CW.
        # Move forward 0.5 m in the local X direction
        # controls.rotateDeltaEuler([0,0,90])
        rospy.loginfo("Attempting to move x by -0.5...")

        # rospy.sleep(10)
        controls.moveDeltaLocal(-0.5, 0, 0)

        rospy.loginfo("Attempting to move x by -0.5...")
        yaw_ctrl.run(-90, step_deg=2, tol_deg=1, rate_hz=300, timeout_s=40)
        rospy.sleep(3)

        print("move")
        controls.moveDeltaLocal(0, -0.5, 0)
        rospy.sleep(3)

        print("sha2")
        yaw_ctrl.run(-90, step_deg=2, tol_deg=1, rate_hz=300, timeout_s=40)
        rospy.sleep(3)

        print("BOom")

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
