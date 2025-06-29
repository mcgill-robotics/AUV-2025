#!/usr/bin/env python3

import rospy
import serial
import numpy as np
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from tf import transformations

RAD_PER_DEG = np.pi / 180.0

#https://docs.waterlinked.com/dvl/dvl-protocol/

"""
WaterLinked DVL ROS driver
Publishes TwistWithCovarianceStamped and PoseWithCovarianceStamped from DVL serial data.
Converts from FRD (sensor) to ENU (AUV/ROS) frame conventions.
"""

def main():
    rospy.init_node("waterlinked_driver")
    pub_twist = rospy.Publisher("/sensors/dvl/twist", TwistWithCovarianceStamped, queue_size=1)
    pub_pose = rospy.Publisher("/sensors/dvl/pose",  PoseWithCovarianceStamped,  queue_size=1)
    port = rospy.get_param("~port")
    baudrate = rospy.get_param("~baudrate")
    quat_var = rospy.get_param("~quat_variance")

    try:
        conn = serial.Serial(port, baudrate=baudrate, timeout=1)
    except serial.serialutil.SerialException:
        rospy.logerr("ERR: cannot open %s", port)
        return

    # reset dead‐reckoning on the DVL
    conn.send_break()
    conn.write(b"wcr\r\n")
    conn.flush()

    # drop initial reset responses
    while not rospy.is_shutdown():
        line = conn.readline().decode(errors="ignore")
        if line.startswith(("wra","wrn")):
            break

    rospy.loginfo("Started DVL driver on %s @ %d baud", port, baudrate)

    while not rospy.is_shutdown():
        line = conn.readline().decode(errors="ignore").strip()
        if not line:
            continue

        #velocity + covariance report (wrz)
        if line.startswith("wrz"):
            # wrz,[vx],[vy],[vz],[valid],[altitude],[fom],[covariance],[time_of_validity],[time_of_transmission],[time],[status]
            # The DVL has +X forward, +Y right, +Z down. The AUV uses ENU: +X forward, +Y left, +Z up. 
            parts = line.replace("*","").split(",")
            try:
                vx, vy, vz = map(float, parts[1:4])
                cov3 = [float(c) for c in parts[7].split(";")]
                # Format of covariance matrix: 
                # [ vx_x,vx_y,vx_z,
                #   vy_x, vy_y, vy_z
                #   vz_x, vz_y, vz_z ] 

            except (ValueError, IndexError):
                rospy.logwarn("Malformed wrz: %s", line)
                continue

            # build 6×6 row‐major covariance with cov3 at upper-left
            cov6 = [0.0]*36
            cov6[0:3] = cov3[0:3] # cov_xx, cov_xy, cov_xz
            cov6[6:9] = cov3[3:6] # cov_yx, cov_yy, cov_yz 
            cov6[12:15] = cov3[6:9] # cov_zx, cov_zy, cov_zz

            msg = TwistWithCovarianceStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "dvl"
            msg.twist.twist.linear.x = vx
            msg.twist.twist.linear.y = -1 * vy
            msg.twist.twist.linear.z = -1 * vz
            msg.twist.covariance = cov6
            pub_twist.publish(msg)
            continue

        # dead‐reckoning pose report (wrp, ~5 Hz) 
        if line.startswith("wrp"):
            #ascii: wrp,[time_stamp],[x],[y],[z],[pos_std],[roll],[pitch],[yaw],[status
            parts = line.replace("*","").split(",")
            try:
                x, y, z = map(float, parts[2:5])
                pos_std = float(parts[5])
                roll, pitch, yaw = map(float, parts[6:9])
            except (ValueError, IndexError):
                rospy.logwarn("Malformed wrp: %s", line)
                continue

            # publish pose
            pose = PoseWithCovarianceStamped()
            pose.header.stamp    = rospy.Time.now()
            pose.header.frame_id = "dvl" # Dead reckoning is done in dvl frame onboard the DVL. 
            pose.pose.pose.position.x = x
            pose.pose.pose.position.y = -1 * y
            pose.pose.pose.position.z = -1 * z

            q = transformations.quaternion_from_euler(roll* RAD_PER_DEG, -1 * pitch * RAD_PER_DEG, -1 * yaw * RAD_PER_DEG)
            pose.pose.pose.orientation.x = q[0]
            pose.pose.pose.orientation.y = q[1]
            pose.pose.pose.orientation.z = q[2]
            pose.pose.pose.orientation.w = q[3]
            
            pos_var = pos_std**2
            cov = [0.0]*36
            cov[0] = pos_var
            cov[7] = pos_var
            cov[14] = pos_var
            cov[21] = quat_var
            cov[28] = quat_var
            cov[35] = quat_var
            pose.pose.covariance = cov

            pub_pose.publish(pose)
            continue

    conn.close()

if __name__ == "__main__":
    main()
