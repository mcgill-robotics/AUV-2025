#!/usr/bin/env python3
import rospy
from substates.utility.controller import Controller

def main():
    """
    Test file for running missions manually during pool tests.
    """

    rospy.init_node("pooltest")

    controls = Controller(rospy.Time(0))    

    try:
        rospy.loginfo("Executing square pattern movement.")
        # negative is CCW, thus positive is CW.
        rospy.loginfo("Attempting to move x by 1.0 and z by -1.0.")
        controls.moveDeltaLocal(1.0, 0, -1.0)
        rospy.loginfo("Completed move successfully.")

        rospy.loginfo("Attempting to yaw by 90 degrees")
        controls.rotate(0.0,0.0,90.0)
        rospy.loginfo("Completed move successfully.")

        rospy.loginfo("Attempting to move y by 1.0.")
        controls.moveDeltaLocal(0, 1.0, 0)
        rospy.loginfo("Completed move successfully.")

        rospy.loginfo("Attempting to yaw by 135 degrees")
        controls.rotate(0.0,0.0,135.0)
        rospy.loginfo("Completed move successfully.")
        
        rospy.loginfo("Attempting to move x and y by -1.0.")
        controls.moveDeltaLocal(-1.0, -1.0, 0.0)
        rospy.loginfo("Completed move successfully.")

        rospy.loginfo("Pool Test completed successfully.")

    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupt received. Stopping movements.")
    finally:
        rospy.loginfo("Shutting down controls…")
        controls.kill()
        
if __name__ == "__main__":
    main()
