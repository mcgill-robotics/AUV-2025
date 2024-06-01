#!/usr/bin/env python3

import rospy
from substates.utility.controller import Controller


if __name__ == '__main__':
     rospy.init_node("pooltest")
     controls = Controller(rospy.Time(0))

     rospy.loginfo("z-Translation  Tests: ")
     rospy.loginfo("moving to: (x=None, y=None, z=-1")
     controls.move([None, None, -1])
     rospy.loginfo("moving to: (x=None, y=None, z=-3")
     controls.move([None, None, -3])
     rospy.loginfo("moving to: (x=None, y=None, z=-0.5")
     controls.move([None, None, -2])
     
     rospy.loginfo("Rotation Tests: ")
     rospy.loginfo("reset rotation: (roll=0, pitch=0, yaw=0")
     controls.rotateEuler([0, 0, 0])
     rospy.loginfo("yaw 180: (roll=0, pitch=0, yaw=180")
     controls.rotateEuler([0, 0, 180])
     rospy.loginfo("reset rotation: (roll=0, pitch=0, yaw=0")
     controls.rotateEuler([0, 0, 0])
     rospy.loginfo("roll 180: (roll=180, pitch=0, yaw=0")
     controls.rotateEuler([180, 0, 0])
     rospy.loginfo("reset rotation: (roll=0, pitch=0, yaw=0")
     controls.rotateEuler([0, 0, 0])
     rospy.loginfo("pitch 180: (roll=0, pitch=180, yaw=0")
     controls.rotateEuler([0, 180, 0])
     rospy.loginfo("reset rotation: (roll=0, pitch=0, yaw=0")