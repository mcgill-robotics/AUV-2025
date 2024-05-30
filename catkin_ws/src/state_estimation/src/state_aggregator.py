#!/usr/bin/env python3

import rospy
import numpy as np
import quaternion
from tf import transformations
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import Pose, Quaternion, Vector3, TransformStamped, Point
from sensors import DepthSensor, IMU, IMUFrontCamera, DVL
from std_msgs.msg import Float64, Int32
from rosgraph_msgs.msg import Clock

DEG_PER_RAD = 180 / np.pi

def swap_main_sensor_message(axis_index, sensor_name):
    global last_warning_message_time

    if rospy.get_time() - last_warning_message_time[axis_index] > sensor_swap_warning_interval:
            last_warning_message_time[axis_index] = rospy.get_time()
            rospy.logwarn("Using {} for {}.".format(sensor_name, axis_names[axis_index]))


def update_state(msg):
    if update_state_on_clock:
        global last_clock_msg_s
        global last_clock_msg_ns
        if last_clock_msg_s == msg.clock.secs and last_clock_msg_ns == msg.clock.nsecs: return
        last_clock_msg_s = msg.clock.secs
        last_clock_msg_ns = msg.clock.nsecs

    position = [None, None, None]
    quaternion = None
    global last_error_message_time

    dvl_reading = dvl.get_reading()
    imu_reading = imu.get_reading()
    imu_front_camera_reading = imu_front_camera.get_reading()
    depth_sensor_reading = depth_sensor.get_reading()

    for axis_index in range(len(position)):
        for sensor_index, sensor in enumerate(sensor_priorities[axis_names[axis_index]]):
            if sensor.get_is_active():
                position[axis_index] = sensor.get_reading()[axis_names[axis_index]]
                if sensor_index != 0:
                    swap_main_sensor_message(axis_index, sensor.get_sensor_name()) 
                break

    for sensor_index, sensor in enumerate(sensor_priorities["orientation"]):
        if sensor.get_is_active():
            sensor_reading = sensor.get_reading()
            reading_quaternion = sensor_reading["quaternion"]
            reading_angular_velocity = sensor_reading["angular_velocity"] 
            
            quaternion = Quaternion(x = reading_quaternion.x, y = reading_quaternion.y, z = reading_quaternion.z, w = reading_quaternion.w)
            euler_dvlref_dvl = transformations.euler_from_quaternion([reading_quaternion.x, reading_quaternion.y, reading_quaternion.z, reading_quaternion.w])
            roll = euler_dvlref_dvl[0] * DEG_PER_RAD
            pitch = euler_dvlref_dvl[1] * DEG_PER_RAD
            yaw = euler_dvlref_dvl[2] * DEG_PER_RAD

            angular_velocity = Vector3(reading_angular_velocity[0], reading_angular_velocity[1], reading_angular_velocity[2])
            swap_main_sensor_message(len(axis_names)-1, sensor_index, sensor.get_sensor_name()) 
            break

    if position[0] is not None and position[1] is not None and position[2] is not None and quaternion is not None:
        pub_x.publish(position[0])
        pub_y.publish(position[1])
        pub_z.publish(position[2])
        pub_theta_x.publish(roll)
        pub_theta_y.publish(pitch)
        pub_theta_z.publish(yaw)
        pub_ang_vel.publish(angular_velocity)
        pose = Pose(Point(x=position[0], y=position[1], z=position[2]), quaternion)
        pub_pose.publish(pose)
        broadcast_auv_pose(pose)
    elif rospy.get_time() - last_error_message_time > 1:
        last_error_message_time = rospy.get_time()
        rospy.logerr("Missing sensor data for proper state estimation! Available states: [ X: {} Y: {} Z: {} Q: {} ]"
            .format(position[0] is not None, position[1] is not None, position[2] is not None, quaternion is not None))

def broadcast_auv_pose(pose):
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "auv_base"
    t.transform.translation.x = pose.position.x
    t.transform.translation.y = pose.position.y
    t.transform.translation.z = pose.position.z 
    t.transform.rotation = pose.orientation
    tf_broadcaster.sendTransform(t)

    t_rot = TransformStamped()
    t_rot.header.stamp = rospy.Time.now()
    t_rot.header.frame_id = "world_rotation"
    t_rot.child_frame_id = "auv_rotation"
    t_rot.transform.translation.x = 0
    t_rot.transform.translation.y = 0
    t_rot.transform.translation.z = 0
    t_rot.transform.rotation = pose.orientation
    tf_broadcaster.sendTransform(t_rot)


if __name__ == '__main__':
    rospy.init_node('state_aggregator')

    pub_pose = rospy.Publisher('/state/pose', Pose, queue_size=1)
    
    pub_x = rospy.Publisher('/state/x', Float64, queue_size=1)
    pub_y = rospy.Publisher('/state/y', Float64, queue_size=1)
    pub_z = rospy.Publisher('/state/z', Float64, queue_size=1)
    pub_theta_x = rospy.Publisher('/state/theta/x', Float64, queue_size=1)
    pub_theta_y = rospy.Publisher('/state/theta/y', Float64, queue_size=1)
    pub_theta_z = rospy.Publisher('/state/theta/z', Float64, queue_size=1)
    pub_ang_vel = rospy.Publisher('/state/angular_velocity', Vector3, queue_size=1)    

    tf_broadcaster = TransformBroadcaster()

    last_error_message_time = rospy.get_time()
    
    depth_sensor = DepthSensor()
    imu = IMU()
    imu_front_camera = IMUFrontCamera()
    dvl = DVL(imu, imu_front_camera)
    
    #by axis, then in order of priority
    sensor_priorities = {
        "x" : [dvl],
        "y" : [dvl],
        "z" : [depth_sensor, dvl],
        "orientation" : [imu, imu_front_camera],
    }

    axis_names = list(sensor_priorities.keys())

    sensor_swap_warning_interval = rospy.get_param("sensor_warning_interval") # secs
    last_warning_message_time = [rospy.get_time()] * 4 # index by axis

    update_state_on_clock = rospy.get_param("update_state_on_clock")

    if update_state_on_clock:
        rospy.Subscriber("/clock", Clock, update_state)
        last_clock_msg_s = None
        last_clock_msg_ns = None
    else:
        timer = rospy.Timer(rospy.Duration(1.0/rospy.get_param("update_rate")), update_state)
        rospy.on_shutdown(timer.shutdown)
    
    rospy.spin()