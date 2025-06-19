#!/usr/bin/env python3

# === Standard Library ===
import math
from math import sin, cos
import numpy as np
from scipy.spatial import KDTree
from scipy.interpolate import splev


# === ROS Core ===
import rospy
import actionlib
import rosbag

import math
from math import cos, sin
import numpy as np
from .functions import *

from geometry_msgs.msg import Pose, Vector3, Vector3Stamped, Wrench, Quaternion, TwistWithCovarianceStamped
from std_msgs.msg import Float64, Bool, Header
from actionlib_msgs.msg import GoalStatus

# === AUV Custom Messages ===
from auv_msgs.msg import (
    EffortAction,
    EffortGoal,
    StateQuaternionAction,
    StateQuaternionGoal,
    ThrusterMicroseconds,
)
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

# === TF Tools ===
import tf2_geometry_msgs
from tf2_ros import Buffer, TransformListener
from tf.transformations import quaternion_from_euler, quaternion_multiply, euler_from_quaternion

# === Local Utilities ===
# TODO: Clean the functions file. Most of the functions there aren't used. 
from .recorder import TopicBagRecorder

# predefined bools so we don't have to write these out everytime we want to get a new goal

do_displace = True
do_not_displace = False
is_local = True
is_not_local = False

class Controller:
    """
    Helper class for the planner. Takes in simple commands, converts them to 
    goals and sends them to the control servers.
    """
    def __init__(self, header_time):
        print("starting controller")
        self.header_time = header_time

        self.x = None
        self.y = None
        self.z = None
        self.orientation = None
        self.clients = []


        # Initialize pub/sub for quaternion controls in controls/quaternion_pid.py
        self.last_quat_error = None
        self.sub_curr_quat_error = rospy.Subscriber("/controls/pid/quat/error", Vector3, lambda msg: setattr(self, "last_quat_error", msg), queue_size=1)
        self.pub_quat_enable = rospy.Publisher( "/controls/pid/quat/enable", Bool, queue_size=1)
        self.pub_quat_setpoint = rospy.Publisher("/controls/pid/quat/setpoint", Quaternion, queue_size=1)

        # Initialize pub for direct torques sent to controls
        self.pub_yaw_torque = rospy.Publisher("/controls/torque/yaw", Float64, queue_size=1)

        # Configure Transform Listener from the world frame
        self.tf_buffer = Buffer()
        TransformListener(self.tf_buffer)
        self.tf_header = Header(frame_id="map")
 
        # Initialize state updates for the controller from state_estimation
        self.sub_pose = rospy.Subscriber("/state/pose", Pose, self.set_pose, queue_size=3)

        # Initialize PID enable topic publishers
        self.pub_x_enable = rospy.Publisher("/controls/pid/x/enable", Bool, queue_size=1)
        self.pub_y_enable = rospy.Publisher("/controls/pid/y/enable", Bool, queue_size=1)
        self.pub_z_enable = rospy.Publisher("/controls/pid/z/enable", Bool, queue_size=1)

        # Initialize PID setpoint topic publishers
        self.pub_x_setpoint = rospy.Publisher("/controls/pid/x/setpoint", Float64, queue_size=10)
        self.pub_y_setpoint = rospy.Publisher("/controls/pid/y/setpoint", Float64, queue_size=10)
        self.pub_z_setpoint = rospy.Publisher("/controls/pid/z/setpoint", Float64, queue_size=10)

        # Initialize relative force topic publishers
        self.pub_surge = rospy.Publisher("/controls/force/surge", Float64, queue_size=1)
        self.pub_sway = rospy.Publisher("/controls/force/sway", Float64, queue_size=1)
        self.pub_heave = rospy.Publisher("/controls/force/heave", Float64, queue_size=1)
        self.pub_roll = rospy.Publisher("/controls/torque/roll", Float64, queue_size=1)
        self.pub_pitch = rospy.Publisher("/controls/torque/pitch", Float64, queue_size=1)
        self.pub_yaw = rospy.Publisher("/controls/torque/yaw", Float64, queue_size=1)

        # Initialize direct force topic publisher
        self.pub_effort = rospy.Publisher("/controls/effort", Wrench, queue_size=1)

        # Initialize global force topic publisher
        self.pub_global_x = rospy.Publisher("/controls/force/global/x", Float64, queue_size=1)
        self.pub_global_y = rospy.Publisher("/controls/force/global/y", Float64, queue_size=1)
        self.pub_global_z = rospy.Publisher("/controls/force/global/z", Float64, queue_size=1)

        # Create publishers for the dropper topic and the claw state topic
        self.claw_state_pub = rospy.Publisher(
            "/actuators/grabber/close", Bool, queue_size=1
        )

        self.pwm_pub = rospy.Publisher("/propulsion/microseconds", ThrusterMicroseconds, queue_size=1)

        # Initialize action clients for actions
        self.EffortClient = actionlib.SimpleActionClient(
            "/controls/server/effort", EffortAction
        )
        self.clients.append(self.EffortClient)
        # print("Waiting for EffortServer to come online...")
        # self.EffortClient.wait_for_server()

        self.StateQuaternionStateClient = actionlib.SimpleActionClient(
            "/controls/server/state", StateQuaternionAction
        )
        self.clients.append(self.StateQuaternionStateClient)


        # Check for missing state information in Controller
        while (
            None
            in [
                self.x,
                self.y,
                self.z,
                self.orientation,
            ]
            and not rospy.is_shutdown()
        ):
            debug_str = "Missing state information for "
            for state_axis, state_axis_name in [
                (self.x, "x"),
                (self.y, "y"),
                (self.z, "z"),
                (self.orientation, "quat."),
            ]:
                if state_axis is None:
                    debug_str += state_axis_name + ", "
            print(debug_str)
            rospy.sleep(1)

                
        TOPICS = {
            "/odometry/filtered": Odometry,
            "/sensors/dvl/twist": TwistWithCovarianceStamped,
            "/sensors/imu/data": Imu,
            "/sensors/depth/z": Float64
        }

        self.recorder = TopicBagRecorder(TOPICS)

        print("All state information received, controller is active.")

    def __del__(self):
        self.kill()

    # Setters

    def set_pose(self, data):
        self.x = data.position.x
        self.y = data.position.y
        self.z = data.position.z
        self.orientation = data.orientation

        q = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        self.current_roll_value, self.current_pitch_value, self.current_yaw_value = euler_from_quaternion(q)

    def wrap(self, err):
        """
        Wraps any angle (radians) into (–π, π]
        """
        return math.atan2(sin(err), cos(err))



    def get_effort_goal(self, dofs):
        """
        Method which returns target goal for the current effort being exerted by controls.
        """

        surge, sway, heave, roll, pitch, yaw = dofs

        goal = EffortGoal()
        goal.effort.force.x = 0 if surge is None else surge
        goal.do_surge = Bool(False) if surge is None else Bool(True)

        goal.effort.force.y = 0 if sway is None else sway
        goal.do_sway = Bool(False) if sway is None else Bool(True)

        goal.effort.force.z = 0 if heave is None else heave
        goal.do_heave = Bool(False) if heave is None else Bool(True)

        goal.effort.torque.x = 0 if roll is None else roll
        goal.do_roll = Bool(False) if roll is None else Bool(True)

        goal.effort.torque.y = 0 if pitch is None else pitch
        goal.do_pitch = Bool(False) if pitch is None else Bool(True)

        goal.effort.torque.z = 0 if yaw is None else yaw
        goal.do_yaw = Bool(False) if yaw is None else Bool(True)

        return goal

    def get_state_goal(self, state, displace, local=is_not_local):
        """
        Method which returns the state goal of the current state action server.

        Note: call with the correct state, displace. and local or unexpected behaviour
        """

        x, y, z, tw, tx, ty, tz = state
        goal = StateQuaternionGoal()

        goal.displace = Bool(displace) 
        goal.local  = Bool(local)

        goal.pose.position.x = 0 if x is None else x
        goal.do_x = Bool(False) if x is None else Bool(True)

        goal.pose.position.y = 0 if y is None else y
        goal.do_y = Bool(False) if y is None else Bool(True)

        goal.pose.position.z = 0 if z is None else z
        goal.do_z = Bool(False) if z is None else Bool(True)

        goal.pose.orientation.w = 1 if tw is None else tw
        goal.pose.orientation.x = 0 if tx is None else tx
        goal.pose.orientation.y = 0 if ty is None else ty
        goal.pose.orientation.z = 0 if tz is None else tz
        goal.do_quaternion = Bool(False) if tz is None else Bool(True)

        return goal

    def preempt_current_action(self):
        """
        Kills action server executions towards specific goals if the goal status is continuing.
        """
        for client in self.clients:
            if client.get_state() in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
                client.cancel_goal()
    
    def enable_pid(self, axis, state):
        """
        Enables PID by switching the boolean in an axis's enable topic
        """
        pub = rospy.Publisher(f"/controls/pid/{axis}/enable", Bool, queue_size=1)
        pub.publish(Bool(state))

    def rotate(self, x: float, y: float, z: float, timeout: float = 20, tol_degrees: float = 1.0):
        '''
        Performs a relative rotation using the AUV's local body axes (rxyz). This follows the Roll-Pitch-Yaw order convention. 
        Positive = counter-clockwise.
        PIDs are kept on after target is reached to maintain target heading. 
        '''
        # 1) Wait for first angle readings, otherwise sleep. Timeout if overtime.       
        err= np.zeros(3)
        start = rospy.Time.now()
        rate = rospy.Rate(20)
        while (self.current_roll_value is None or
            self.current_pitch_value is None or
            self.current_yaw_value is None) and not rospy.is_shutdown():
            if (rospy.Time.now() - start).to_sec() > timeout:
                raise RuntimeError("Topic missing, aborting process...")
            rate.sleep()

        # 2) Get current attitude
        current_q = np.array([self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w])

        tol = math.radians(tol_degrees) # Calculate tolerance

       # 3) Calculate change in attitude
        delta_q = quaternion_from_euler(math.radians(x), math.radians(y), math.radians(z), axes = "rxyz") # rxyz is used since we want Intrinsic (body-frame) rotation: rotate around rotating AUV axes.

        # 4) Publish correct quaternion setpoint to the controls server
        target_q = quaternion_multiply(current_q, delta_q)
        qt = Quaternion(x=target_q[0], y=target_q[1], z=target_q[2], w=target_q[3])
        self.pub_quat_setpoint.publish(qt)

        # 5)enable the x,y,z, and quaternion PIDs
        self.enable_pid("x",    True)
        self.enable_pid("y",    True)
        self.enable_pid("z",    True)
        self.enable_pid("quat", True)

        # 6) Wait for /controls/pid/quat/error topic to start publishing...
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            # Break the process if the error is below tolerance level
            if self.last_quat_error is not None:
                err = np.array([self.last_quat_error.x, self.last_quat_error.y, self.last_quat_error.z])
                if np.linalg.norm(err) < tol:
                    break

            # Send warning if timeout.
            if (rospy.Time.now() - start).to_sec() > timeout:
                err_norm = np.linalg.norm(err)
                rospy.logwarn("rotate timed out: %.1f error", err_norm)       


    def rotateYaw(self, delta_degrees: float, timeout: float = 10.0,tol_degrees: float = 1.0):
        """
        Rotate in place by delta_degrees (positive = counter-clockwise).
        Blocks until the (yaw-error) < tol_degrees or timeout expires.
        """



        # 1) Wait for first yaw reading, otherwise sleep. Timeout if overtime.
        yaw_err = 0.0
        start = rospy.Time.now()
        rate = rospy.Rate(20)
        while self.current_yaw_value is None and not rospy.is_shutdown():
            if (rospy.Time.now() - start).to_sec() > timeout:
                raise RuntimeError("Topic missing: /state/theta/z, aborting process...")
            rate.sleep()

        # 2) Compute target yaw
        begin = self.current_yaw_value
        target = self.wrap(begin + math.radians(delta_degrees))

        tol= math.radians(tol_degrees)  # calculate tolerance 
        rospy.loginfo(f"{target}")
        q = quaternion_from_euler(0, 0, target, axes="sxyz")

        rospy.loginfo(f"Target rotation angle: {target}")
        rospy.loginfo(f"Target quaterion angle: {q}")

        # 3) Publish correct quaternion setpoint to the controls server
        qt = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]) # use Quaternion msg type
        print("qt", qt)
        self.pub_quat_setpoint.publish(qt)

        # 4)enable the x,y,z, and quaternion PIDs
        # Note: this is done for consistency purposes, it is hard for the AUV to rotate while also moving
        self.enable_pid("x",    True)
        self.enable_pid("y",    True)
        self.enable_pid("z",    True)
        self.enable_pid("quat", True)

        # 5) Wait for /controls/pid/quat/error topic to start publishing...
        start = rospy.Time.now()
        while not rospy.is_shutdown():
            # Break the process if the error is below tolerance level
            if self.last_quat_error is not None:
                err = np.array([self.last_quat_error.x, self.last_quat_error.y, self.last_quat_error.z])
                if np.linalg.norm(err) < tol:
                    break

            # Quit process if timeout.
            if (rospy.Time.now() - start).to_sec() > timeout:
                err_norm = np.linalg.norm(err)
                rospy.logwarn("rotate timed out: %.1f error", err_norm)  

    # TODO: Add documentation to everything below this
    def state(self, pos, ang):
        x, y, z = pos
        if any(x is None for x in ang) and any(x is not None for x in ang):
            raise ValueError(
                "Invalid state goal: quaternion cannot have a combination of None and valid values. Goal received: {}".format(
                    ang
                )
            )
        w, wx, wy, wz = ang
        goal_state = self.get_state_goal([x, y, z, w, wx, wy, wz], do_not_displace)
        self.StateQuaternionStateClient.send_goal_and_wait(goal_state)

    def stateDelta(self, pos, ang):
        x, y, z = pos
        if any(x is None for x in ang) and any(x is not None for x in ang):
            raise ValueError(
                "Invalid stateDelta goal: quaternion cannot have a combination of None and valid values. Goal received: {}".format(
                    ang
                )
            )
        w, wx, wy, wz = ang
        goal_state = self.get_state_goal([x, y, z, w, wx, wy, wz], do_displace)
        self.StateQuaternionStateClient.send_goal_and_wait(goal_state)


    def stateDeltaEuler(self, pos, ang):
        wx, wy, wz = ang
        if wx is None:
            wx = 0
        if wy is None:
            wy = 0
        if wz is None:
            wz = 0
        self.stateDelta(pos, euler_to_quaternion(wx, wy, wz))

    # move to setpoint
    def move(self, pos, face_destination=False):
        x, y, z = pos

        goal_state = self.get_state_goal(
            [x, y, z, None, None, None, None], do_not_displace
        )

        x = self.x if x is None else x
        y = self.y if y is None else y
        if face_destination and math.sqrt(x**2 + y**2) > 0.5:
            yaw_towards_destination = vectorToYawDegrees(x - self.x, y - self.y)
            self.rotateEuler((0, 0, yaw_towards_destination))

        self.StateQuaternionStateClient.send_goal_and_wait(goal_state)

    # move by this amount in world space
    def moveDelta(self, delta, face_destination=False):
        x, y, z = delta

        goal_state = self.get_state_goal([x, y, z, None, None, None, None], do_displace)

        x = 0 if x is None else x
        y = 0 if y is None else y
        if face_destination and math.sqrt(x**2 + y**2) > 0.5:
            yaw_towards_destination = vectorToYawDegrees(x, y)
            self.rotateEuler((0, 0, yaw_towards_destination))

        self.StateQuaternionStateClient.send_goal_and_wait(goal_state)


    def moveDeltaLocal(self, delta_x, delta_y, delta_z, tolerance=0.05, timeout=30):
        """
        Translate the AUV by a specified amount in x, y, z relative to the auv frame.
        """
        
        rospy.sleep(1.0)  # Let odometry settle

        # TODO: Make this map after SLAM
        # 1) Compute target in odom frame
        target_x = self.x + delta_x
        target_y = self.y + delta_y
        target_z = self.z + delta_z

        print(f"Target x: {target_x:.3f}, y: {target_y:.3f}, z: {target_z:.3f}")

        # 2) Enable positional PID control
        self.enable_pid("x", True)
        self.enable_pid("y", True)
        self.enable_pid("z", True)

        self.pub_x_setpoint.publish(target_x)
        self.pub_y_setpoint.publish(target_y)
        self.pub_z_setpoint.publish(target_z)

        rate = rospy.Rate(20)
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < timeout and not rospy.is_shutdown():
            err_x = abs(self.x - target_x)
            err_y = abs(self.y - target_y)
            err_z = abs(self.z - target_z)

            # rospy.loginfo(f"err_x: {err_x:.3f}, err_y: {err_y:.3f}")
            if err_x < tolerance and err_y < tolerance and err_z < tolerance:
                break

            rate.sleep()

#TODO: Upgrade this to use make_splprep and BSpline objects when we have newer version of SciPy installed

    def LoS(self, Path, lookahead: float, n: int = 1000, freq: int = 10):
        ''' 
        Controls the AUV along the given Path using Line-of-Sight.
    
        Args:
            Path: Tuple (tck) from scipy.interpolate.splprep
            lookahead: Lookahead distance in meters
            n: Number of sampled points along the path
            freq: Loop rate in Hz
        '''
        tck = Path  # Explicit renaming for clarity
        us = np.linspace(0, 1, n)
        pts = np.array(splev(us, tck)).T  # Evaluate and transpose to (n, 3)

        arc_lengths = np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))
        arc_lengths = np.insert(arc_lengths, 0, 0)
        total_length = arc_lengths[-1]

        tree = KDTree(pts)
        rate = rospy.Rate(freq)

        self.enable_pid("x", True)
        self.enable_pid("y", True)
        self.enable_pid("z", True)
        self.enable_pid("quat", True)

        i_target = 0
        while not rospy.is_shutdown():
            # Ensure state is ready
            if None in [self.x, self.y, self.z]:
                rospy.logwarn("Waiting for state estimate...")
                rate.sleep()
                continue

            current_pos = np.array([self.x, self.y, self.z])
            _, i_closest = tree.query(current_pos)
            s_closest = arc_lengths[i_closest]

            s_target = min(s_closest + lookahead, total_length)
            i_target = np.searchsorted(arc_lengths, s_target)
            i_target = min(i_target, n - 1)
            rospy.loginfo(f" Step: {i_target}")

            target_x, target_y, target_z = pts[i_target]
            self.pub_x_setpoint.publish(target_x)
            self.pub_y_setpoint.publish(target_y)
            self.pub_z_setpoint.publish(target_z)

            if i_target == n - 1:
                rospy.loginfo("Target reached.")
                break

            rate.sleep()
    
    
    
    
    def torque(self, vel):
        """
        Sets a torque value (x, y, z) and sets this as the goal in the effort server
        """
        x, y, z = vel
        goal = self.get_effort_goal([None, None, None, x, y, z])
        self.EffortClient.send_goal(goal)

    def forceLocal(self, vel):
        """
        Sets a positonal effort force in the local reference frame.

        Note: z is unaffected bu this method (always heaving)
        """

        x, y = vel
        goal = self.get_effort_goal([x, y, None, None, None, None])
        self.EffortClient.send_goal(goal)

    def kill(self):
        """
        Kills all the pid efforts active and removes all forces being applied on thrusters
        """

        self.preempt_current_action()

        goal = self.get_effort_goal([0, 0, 0, 0, 0, 0])
        self.EffortClient.send_goal(goal)
        self.pub_x_enable.publish(Bool(False))
        self.pub_y_enable.publish(Bool(False))
        self.pub_z_enable.publish(Bool(False))
        self.pub_quat_enable.publish(Bool(False))
        
        # Disable all active PIDs
        self.enable_pid("x", False)
        self.enable_pid("y", False)
        self.enable_pid("z", False)
        self.enable_pid("quat", False)

        rospy.logwarn("Killing all thrusters and disabling PIDs...")

        # Iterate during timeout
        start = rospy.get_time()
        while rospy.get_time() - start < 5:
            self.pub_surge.publish(0)
            self.pub_sway.publish(0)
            self.pub_heave.publish(0)
            self.pub_roll.publish(0)
            self.pub_pitch.publish(0)
            self.pub_yaw.publish(0)
            self.pub_global_x.publish(0)
            self.pub_global_y.publish(0)
            self.pub_global_z.publish(0)

            zero_wrench = Wrench()
            zero_wrench.force.x = 0
            zero_wrench.force.y = 0
            zero_wrench.force.z = 0
            zero_wrench.torque.x = 0
            zero_wrench.torque.y = 0
            zero_wrench.torque.z = 0
            self.pub_effort.publish(zero_wrench)
            self.pwm_pub.publish(ThrusterMicroseconds([1500] * 8))

    # TODO: Debug and verify with tests
    def freeze_pose(self):
        goal = self.get_state_goal(
            [
                self.x,
                self.y,
                self.z,
                self.orientation.w,
                self.orientation.x,
                self.orientation.y,
                self.orientation.z,
            ],
            do_not_displace,
        )
        self.StateQuaternionStateClient.send_goal_and_wait(goal)

    # TODO: Debug and verify with tests
    def freeze_position(self):
        goal = self.get_state_goal(
            [self.x, self.y, self.z, None, None, None, None], do_not_displace
        )
        self.StateQuaternionStateClient.send_goal_and_wait(goal)

    # TODO: Debug and verify with tests
    def freeze_rotation(self):
        goal = self.get_state_goal(
            [
                None,
                None,
                None,
                self.orientation.w,
                self.orientation.x,
                self.orientation.y,
                self.orientation.z,
            ],
            do_not_displace,
        )
        self.StateQuaternionStateClient.send_goal_and_wait(goal)

    # TODO: Debug and verify with tests
    def flatten(self):
        """
        Computes a new goal which flattens the AUV such that only the yaw component is maintained.
        """

        orientation = np.quaternion(
            self.orientation.w,
            self.orientation.x,
            self.orientation.y,
            self.orientation.z,
        )
        v = orientation * np.quaternion(0, 1, 0, 0) * orientation.conjugate()
        v = np.array([v.x, v.y])
        v = v / np.linalg.norm(v)
        forward = np.array((1, 0))
        angle = math.acos(v.dot(forward))
        if v[0] < 0:
            angle *= -1
        final = np.quaternion(math.cos(angle / 2), 0, 0, math.sin(angle / 2))

        goal = self.get_state_goal(
            [None, None, None, final.w, final.x, final.y, final.z], do_not_displace
        )
        self.StateQuaternionStateClient.send_goal_and_wait(goal)

    # TODO: Debug and verify with tests
    def open_claw(self):
        self.claw_state_pub.publish(Bool(True))

    # TODO: Debug and verify with tests
    def close_claw(self):
        self.claw_state_pub.publish(Bool(False))