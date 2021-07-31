import rospy
import smach
import actionlib # No ActionServer implemented yet, we might want TODO this eventually

from std_msgs.msg import Float64, Float32MultiArray

class TouchBuoy(smach.State):
    '''
    This state assumes that we have already navigated to the BUOY, and it is centered 
    in the viewframe. 
    The 'lateral' (heave and sway) PIDs from the previous state will still be running, such that the
    buoy will be maintined in the center of the viewframe. 

    This state is responsible for
        1. launching the KNN detector from the cv package
        2. Listening to the output topics of this package to await image detections
        3. Upon recognition of the target, surging to touch it
        4. Transitioning into the next task ('navigate to the surface') 
    
    This state is making some...potentially nasty assumptions. THe one that I am worried about is
       1. If the "wrong" image is on the BUOY, we will need to wait untill the buoy rotates. 
          Its not clear to me that just leaving the PIDS on will robustly handle this . We'll see in pool testing 
    
    The syntax for launching the launch file comes from
    http://wiki.ros.org/roslaunch/API%20Usage 

    '''
    def __init__(self):
        #Define the smach transitions that are possible
        smach.State.__init__(self, outcomes=['TouchedTheBuoy'])

        self.SURGE_TIME       = 2    # Units of seconds
        self.SURGE_STRENGTH   = 2     # A small number, to be changed during pool testing
        self.initial_time     = None  # To be overwritten when the target is recognized for the first time
        self.target_acquired  = False # To know if we need to start the timer!
        self.finished_surging = False

        self.knn_sub               = rospy.Subscriber('/objects', Float32MultiArray, self.knn_cb)
        self.surge_magnitude_pub   = rospy.Publisher('/controls/superimposer/surge'   , Float64 , queue_size=1)
        
    def knn_cb(self, objectArray):
        '''
        WE shouuuuuuld already have the buoy in the center of the viewframe due to the
        Colour thresholding PIDs. all we need to do is surge.
        As of now, this does NOT wait for several consecutive counts, and surges immediately 
        upon recognizing a target.
        '''
        print('Data on topic : {}'.format(objectArray))
        #Only do something if the data on the topic isn't empty
        if len(objectArray.data)> 0:
            if not self.target_acquired: 
                self.initial_time = time.time()
                self.surge_toward_target()
                self.target_acquired = True
            if self.target_acquired == True:
                # The surge function hangs, and does all the surging! 
                # If we're already surging, we just want this to do nothing!
                return

    def surge_toward_target(self):
        while time.time()- self.initial_time < self.SURGE_TIME : 
            self.surge_magnitude_pub.publish(self.SURGE_STRENGTH)
            rospy.sleep(0.1)
        self.finished_surging = True
        return


    def execute(self, userdata):
        # Launch the knn Detector Launch file
        # This wild magic is from the link in the comment above

        #rospy.init_node('en_Mapping', anonymous=True)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        knn_launch = roslaunch.parent.ROSLaunchParent(uuid, 
                    ["/home/tommy/robotics/AUV-2020/catkin_ws/src/cv/launch/knnDetector.launch"])
        knn_launch.start()
        rospy.loginfo("knn launcher started")
        #Wait untill surging is finished
        while not self.finished_surging:
            #do nothing! Wait for the KNN detector to publish something on the /objects topic!
            pass
        
        #knnDetector_process.stop()
        knn_launch.shutdown()
        return 'TouchedTheBuoy'