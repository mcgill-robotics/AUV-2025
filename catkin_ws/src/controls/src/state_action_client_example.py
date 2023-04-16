#!/usr/bin/env python3

import rospy
import actionlib
from geometry_msgs.msg import Point, Pose
from std_msgs.msg import Float64, Bool
from auv_msgs.msg import StateAction, StateGoal, SuperimposerAction, SuperimposerGoal

if __name__ == '__main__':
    print("starting test")
    rospy.init_node('example')

    goal = SuperimposerGoal()
    goal.effort.force.x = 1
    goal.effort.force.y = 1
    goal.effort.force.z = 1
    goal.effort.torque.x = 1
    goal.effort.torque.y = 1
    goal.effort.torque.z = 1
    goal.do_surge, goal.do_sway, goal.do_heave, goal.do_roll, goal.do_pitch, goal.do_yaw = [Bool(True)]*6
    goal.displace = Bool(False)
    server = actionlib.SimpleActionClient('superimposer_local_server', SuperimposerAction)
    server.wait_for_server()
    server.send_goal(goal)
    rospy.sleep(5)
    server.cancel_goal()

    # position = Point()
    # rotation = Point()
    # position.x = 1
    # position.y = 1 
    # position.z = 1
    # rotation.x = 1
    # rotation.y = 1
    # rotation.z = 1

    # pub_pos = rospy.Publisher("pose",Pose,queue_size=50)
    # pub_tx = rospy.Publisher("state_theta_x",Float64,queue_size=50)
    # pub_ty = rospy.Publisher("state_theta_y",Float64,queue_size=50)
    # pub_tz = rospy.Publisher("state_theta_z",Float64,queue_size=50)
    # pub_z = rospy.Publisher("state_z",Float64,queue_size=50)

    # p = Pose()
    # p.position.x = 0
    # p.position.y = 0
    # p.position.z = 0
    # p.orientation.w = 1
    # p.orientation.x = 1
    # p.orientation.y = 0
    # p.orientation.z = 0

    # rospy.sleep(4)

    # pub_pos.publish(p)
    # pub_z.publish(Float64(0))
    
    # pub_tx.publish(Float64(0))
    # pub_ty.publish(Float64(0))
    # pub_tz.publish(Float64(0))

    # server = actionlib.SimpleActionClient('state_server', StateAction)
    # server.wait_for_server()
    # goal = StateGoal()
    # goal.position = position
    # goal.rotation = rotation
    # goal.do_x, goal.do_y, goal.do_z, goal.do_theta_x, goal.do_theta_y, goal.do_theta_z = [Bool(True)]*6
    # goal.displace = Bool(False)
    # print(goal)
    # server.send_goal(goal)
    # rospy.sleep(5)
    # print("cancel goal")
    # server.cancel_goal()
    # server.send_goal_and_wait(goal)
    # server.cancel_goal()