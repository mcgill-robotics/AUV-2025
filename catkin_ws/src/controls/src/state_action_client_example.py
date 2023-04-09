import rospy
import actionlib
from geometry_msgs.msg import Point
from auv_msgs.msg import StateAction, StateGoal

if __name__ == '__main__':
    rospy.init_node('example')
    position = Point()
    rotation = Point()
    position.x = 0
    position.y = 0 
    position.z = 0
    rotation.x = 0
    rotation.y = 0
    rotation.z = 0

    server = actionlib.SimpleActionClient('state_server', StateAction)
    server.wait_for_server()
    goal = StateGoal()
    goal.position = position
    goal.rotation = rotation
    server.send_goal(goal)
    #server.send_goal_and_wait(goal)
    #server.cancel_goal()