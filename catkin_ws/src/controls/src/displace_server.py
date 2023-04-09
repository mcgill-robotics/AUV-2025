import rospy
import actionlib
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose
from auv_msgs.msg import StateAction, StateFeedback, StateResult
from std_msgs.msg import Float64

class DisplaceServer():

    def __init__(self):
        self.server = actionlib.SimpleActionServer('displace_server', StateAction, execute_cb= self.callback, auto_start = False)

        self.pub_z = rospy.Publisher('z_setpoint', Float64, queue_size=50)
        self.pub_theta_x = rospy.Publisher('theta_x_setpoint', Float64, queue_size=50)
        self.pub_theta_y = rospy.Publisher('theta_y_setpoint', Float64, queue_size=50)
        self.pub_theta_z = rospy.Publisher('theta_z_setpoint', Float64, queue_size=50)
        
        self.server.start()

        self.position = None
        self.theta_x = None
        self.theta_y = None
        self.theta_z = None

        self.sub = rospy.Subscriber("pose",Pose,self.set_position)
        self.sub = rospy.Subscriber("state_theta_x",Float64,self.set_theta_x)
        self.sub = rospy.Subscriber("state_theta_y",Float64,self.set_theta_y)
        self.sub = rospy.Subscriber("state_theta_z",Float64,self.set_theta_z)

    def set_position(self,data):
        #print("updated pose")
        self.position = data.position
    
    def set_theta_x(self,data):
        self.theta_x = data.data
    def set_theta_y(self,data):
        self.theta_y = data.data
    def set_theta_z(self,data):
        self.theta_z = data.data


    def callback(self, goal):
        #print("got a message")
        # set the PIDs
        #print(goal.pose)
        goal_x = goal.position.x + self.position.x
        goal_y = goal.position.y + self.position.y
        goal_z = goal.position.z + self.position.z

        goal_theta_x = goal.rotation.x + self.theta_x
        goal_theta_y = goal.rotation.y + self.theta_y
        goal_theta_z = goal.rotation.z + self.theta_z

        goal_position = Point()
        goal_position.x = goal_x
        goal_position.y = goal_y
        goal_position.z = goal_z

        goal_rotation = Point()
        goal_rotation.x = goal_theta_x
        goal_rotation.y = goal_theta_y
        goal_rotation.z = goal_theta_z

        self.publish_setpoints(goal_position,goal_rotation)

        # monitor when reached pose
        self.wait_for_settled()
        
        result = StateResult()
        result.status.data = True
        rospy.loginfo("Succeeded")
        self.server.set_succeeded(result)

    def wait_for_settled(self,goal_position,goal_rotation):
        interval = 0.3

        settled = False

        while not settled:
            start = time.time()
            while self.check_status(goal_position,goal_rotation):
                if(time.time() - start < interval):
                    settled = True
                    break
                rospy.sleep(0.01)

    def check_status(self,position,rotation):
        if(self.position == None or self.theta_x == None or self.theta_y == None or self.theta_z == None):
            return False

        tolerance_position = 0.5
        tolerance_orientation = 1

        #x_diff = abs(self.position.x - position.x) <= tolerance_position
        #y_diff = abs(self.position.y - position.y) <= tolerance_position
        z_diff = abs(self.position.z - position.z) <= tolerance_position
        theta_x_diff = abs(self.theta_x - rotation.x) <= tolerance_orientation
        theta_y_diff = abs(self.theta_y - rotation.y) <= tolerance_orientation
        theta_z_diff = abs(self.theta_z - rotation.z) <= tolerance_orientation

        return z_diff and theta_x_diff and theta_y_diff and theta_z_diff # and x_diff and y_diff


    def publish_setpoints(self,position,rotation):
        #self.pub_x.Publish(Float64(position.x))
        #self.pub_y.Publish(Float64(position.y))
        self.pub_z.publish(Float64(position.y))
        self.pub_theta_x.publish(Float64(rotation.x))
        self.pub_theta_y.publish(Float64(rotation.y))
        self.pub_theta_z.publish(Float64(rotation.z))
        #print("published setpoints")