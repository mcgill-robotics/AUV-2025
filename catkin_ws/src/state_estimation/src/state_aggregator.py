#!/usr/bin/env python3

import rospy
import numpy as np
import quaternion
from tf import transformations
from tf2_ros import TransformBroadcaster
import math

from geometry_msgs.msg import Pose, Quaternion, Vector3, TransformStamped, Point
from sensors import DepthSensor, IMU, DVL
from std_msgs.msg import Float64

DEG_PER_RAD = 180 / np.pi

def update_state(_):    
    x = None
    y = None
    z = None
    quaternion = None
    ds_z = None
    dvl_z = None
    i = 0
    global last_error_message_time

    for sensor in sensor_priorities["x"]:
        if sensor.isActive():
            x = sensor.x
            break
    for sensor in sensor_priorities["y"]:
        if sensor.isActive():
            y = sensor.y
            break
    for sensor in sensor_priorities["z"]:
        if sensor.isActive():
            if sensor.sensor_name == "depth_sensor":
                ds_z = sensor.z
                if dvl_z is not None:
                    offset = math.abs(ds_z - dvl_z)
                    i += 1
                    ds_z -= math.exp(-i)*offset
                z = ds_z
            else:
                dvl_z = sensor.z
                if ds_z is not None:
                    offset = math.abs(ds_z - dvl_z)
                    dvl_z -= offset
                z = dvl_z
                break
    for sensor in sensor_priorities["orientation"]:
        if sensor.isActive():
            np_quaternion = np.array([sensor.q_nwu_auv.x, sensor.q_nwu_auv.y, sensor.q_nwu_auv.z, sensor.q_nwu_auv.w])
            quaternion = Quaternion(x = sensor.q_nwu_auv.x, y = sensor.q_nwu_auv.y, z = sensor.q_nwu_auv.z, w = sensor.q_nwu_auv.w)
            roll = transformations.euler_from_quaternion(np_quaternion, 'rxyz')[0] * DEG_PER_RAD
            pitch = transformations.euler_from_quaternion(np_quaternion, 'ryxz')[0] * DEG_PER_RAD
            yaw = transformations.euler_from_quaternion(np_quaternion, 'rzyx')[0] * DEG_PER_RAD
            angular_velocity = Vector3(sensor.angular_velocity[0], sensor.angular_velocity[1], sensor.angular_velocity[2])
            break
    if x is not None and y is not None and z is not None and quaternion is not None:
        pub_x.publish(x)
        pub_y.publish(y)
        pub_z.publish(z)
        pub_theta_x.publish(roll)
        pub_theta_y.publish(pitch)
        pub_theta_z.publish(yaw)
        pub_ang_vel.publish(angular_velocity)
        pose = Pose(Point(x=x, y=y, z=z), quaternion)
        pub_pose.publish(pose)
        broadcast_auv_pose(pose)
    elif rospy.get_time() - last_error_message_time > 1:
        last_error_message_time = rospy.get_time()
        rospy.logerr("Missing sensor data for proper state estimation! Available states: [ X: {} Y: {} Z: {} Q: {} ]".format(x is not None, y is not None, z is not None, quaternion is not None))

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

    pub_pose = rospy.Publisher('pose', Pose, queue_size=1)
    pub_x = rospy.Publisher('state_x', Float64, queue_size=1)
    pub_y = rospy.Publisher('state_y', Float64, queue_size=1)
    pub_z = rospy.Publisher('state_z', Float64, queue_size=1)
    pub_theta_x = rospy.Publisher('state_theta_x', Float64, queue_size=1)
    pub_theta_y = rospy.Publisher('state_theta_y', Float64, queue_size=1)
    pub_theta_z = rospy.Publisher('state_theta_z', Float64, queue_size=1)
    pub_ang_vel = rospy.Publisher('angular_velocity', Vector3, queue_size=1)
    tf_broadcaster = TransformBroadcaster()

    last_error_message_time = rospy.get_time()
    
    depth_sensor = DepthSensor()
    imu = IMU()
    dvl = DVL(imu)
    
    #by axis, then in order of priority
    sensor_priorities = {
        "x" : [dvl],
        "y" : [dvl],
        "z" : [depth_sensor, dvl],
        "orientation" : [imu, dvl],
    }

    timer = rospy.Timer(rospy.Duration(1.0/rospy.get_param("~update_rate")), update_state)
    rospy.on_shutdown(timer.shutdown)
    
    rospy.spin()
