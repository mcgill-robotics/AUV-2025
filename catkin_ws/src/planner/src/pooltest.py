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

    controls = Controller(rospy.Time(0))    
    linear_controls = LinearController()
    yaw_ctrl = YawRamp()

    try:
        rospy.loginfo("Executing square pattern movement.")
        # negative is CCW, thus positive is CW.
        rospy.loginfo("Attempting to move x and y by 2.0.")
        linear_controls.moveDeltaLocal(2.0, 0, 0)
        rospy.loginfo("Completed move successfully.")

        rospy.loginfo("Attempting to rotate yaw by 90 degrees counter-clockwise then 90 degrees clockwise")
        yaw_ctrl.run(90)
        rospy.loginfo("Completed move successfully.")

        linear_controls.moveDeltaLocal(0, -2, 0)

        rospy.loginfo("Attempting to rotate yaw  by 90 degrees counter-clockwise then 90 degrees clockwise")
        yaw_ctrl.run(90) 
        rospy.loginfo("Completed move successfully.")
        
        rospy.loginfo("Attempting to move x and y by 2.0.")
        linear_controls.moveDeltaLocal(2.0, -2.0, 0)
        rospy.loginfo("Completed move successfully.")

        rospy.loginfo("Pool Test completed successfully.")

    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt received. Stopping movements.")
    finally:
        rospy.loginfo("Shutting down controls…")
        linear_controls.kill()
        yaw_ctrl.kill()
if __name__ == "__main__":
    main()
