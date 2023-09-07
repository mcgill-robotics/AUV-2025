#!/usr/bin/env python3

import rospy
import numpy as np
import quaternion
from tf import transformations
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import Pose, Quaternion, Vector3, TransformStamped
from sensors import DepthSensor, IMU, DVL
from std_msgs.msg import Float64

def update_state():    
    # X
    for sensor in sensor_priorities["x"]:
        if sensor.isActive():
            x = sensor.x
            break
    # Y
    for sensor in sensor_priorities["y"]:
        if sensor.isActive():
            y = sensor.y
            break
    # Z
    for sensor in sensor_priorities["z"]:
        if sensor.isActive():
            z = sensor.z
            break
    # ORIENTATION
    for sensor in sensor_priorities["orientation"]:
        if sensor.isActive():
            quaternion = sensor.quaternion
            roll, pitch, yaw = sensor.roll, sensor.pitch, sensor.yaw
            angular_velocity = sensor.angular_velocity
            break
    # POSE
    pose = Pose(Point(x=x, y=y, z=z), orientation)
    # PUBLISH/BROADCAST STATE
    pub_pose.publish(pose)
    broadcast_auv_pose(pose)
    pub_x.publish(x)
    pub_y.publish(y)
    pub_z.publish(z)
    pub_theta_x.publish(roll)
    pub_theta_y.publish(pitch)
    pub_theta_z.publish(yaw)
    pub_ang_vel.publish(angular_velocity)
    # UPDATE SENSORS WITH MOST ACCURATE POSE ESTIMATE
    dvl.updateXYZ(x if dvl.x != x else None, y if dvl.y != y else None, z if dvl.z != z else None)
    imu.updateXYZ(x if imu.x != x else None, y if imu.y != y else None, z if imu.z != z else None)

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
    
    dvl = DVL()
    depth_sensor = DepthSensor()
    imu = IMU()
    
    #by axis, then in order of priority
    sensor_priorities = {
        "x" : [dvl, imu],
        "y" : [dvl, imu],
        "z" : [depth_sensor, dvl, imu],
        "orientation" : [imu, dvl],
    }

    timer = rospy.Timer(rospy.Duration(1.0/rospy.get_param("~update_rate")), update_state)
    rospy.on_shutdown(timer.shutdown)