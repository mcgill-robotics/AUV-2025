#!/usr/bin/env python3
import rospy
from std_msgs.msg import Bool

def test_claw():
    rospy.init_node("claw_test_node", anonymous=True)
    claw_pub = rospy.Publisher("/actuators/grabber/close", Bool, queue_size=1)
    rospy.sleep(1)  # Give ROS time to set up

    print("Closing claw...")
    claw_pub.publish(Bool(True))
    rospy.sleep(3)

    print("Opening claw...")
    claw_pub.publish(Bool(False))
    rospy.sleep(3)

    print("Test complete.")

if __name__ == "__main__":
    test_claw()
