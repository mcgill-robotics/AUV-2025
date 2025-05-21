#!/usr/bin/env python3
import rospy
from substates.utility.controller import Controller
from substates.utility.yaw_controller import YawRamp
from substates.utility.linear_controller import LinearController

def main():
    """
    Test file for running missions manually during pool tests.
    """

    rospy.init_node("pooltest")

    controls = Controller(rospy.Time(0))     # Only need one Controller instantiation
    linear_controls = LinearController()
    yaw_ctrl = YawRamp()
    try:
        rospy.loginfo("Executing square pattern movement.")
        # negative is CCW, thus positive is CW.
        # Move forward 0.5 m in the local X direction
        # controls.rotateDeltaEuler([0,0,90])
        rospy.loginfo("Attempting to move x and y by 2.0.")
        linear_controls.moveDeltaLocal(2.0, -2.0, 0)
        rospy.loginfo("Completed move successfully.")

        rospy.loginfo("Attempting to rotate yaw by 90 degrees counter-clockwise then 90 degrees clockwise")
        controls.rotateYaw(90, timeout = 5.0, tol_degrees = 0.5)
        rospy.loginfo("Completed move successfully.")

        rospy.loginfo("Attempting to rotate yaw  by 90 degrees counter-clockwise then 90 degrees clockwise")
        controls.rotateYaw(50, timeout = 5.0, tol_degrees = 0.5)
        rospy.loginfo("Completed move successfully.")
        
        rospy.loginfo("Attempting to move x and y by 2.0.")
        linear_controls.moveDeltaLocal(2.0, -2.0, 0)
        rospy.loginfo("Completed move successfully.")

        # rospy.loginfo("Attempting to move y by -2.0.")
        # controls.moveDeltaLocal(0, -2.0, 0)
        # rospy.loginfo("Completed move successfully.")

        # rospy.loginfo("Attempting to use the YawController to perform step-wise rotations.")
        # yaw_ctrl.run(-90, step_deg=2, tol_deg=1, rate_hz=300, timeout_s=40)
        # rospy.loginfo("Completed move successfully.")

        rospy.loginfo("Pool Test completed successfully.")

        # …and so on if you uncomment the rest

    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt received. Stopping movements.")
    finally:
        rospy.loginfo("Shutting down controls…")
        controls.kill()

if __name__ == "__main__":
    main()
