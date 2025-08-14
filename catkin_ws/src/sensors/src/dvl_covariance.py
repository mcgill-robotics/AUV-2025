#!/usr/bin/env python3

import rospy
import serial
import numpy as np
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from tf import transformations


#https://docs.waterlinked.com/dvl/dvl-protocol/



RAD_PER_DEG = np.pi / 180.0

def main():
    rospy.init_node("waterlinked_driver")

    pub_twist = rospy.Publisher("/sensors/dvl/twist", TwistWithCovarianceStamped, queue_size=10)
    pub_pose  = rospy.Publisher("/sensors/dvl/pose",  PoseWithCovarianceStamped,  queue_size=10)

    port          = rospy.get_param("~port")
    baudrate      = rospy.get_param("~baudrate", 115200)
    fom_threshold = rospy.get_param("~fom_threshold", 25.0)
    frame_id      = rospy.get_param("~frame_id", "dvl")

    # Covariance knobs
    big_var_ang   = rospy.get_param("~big_var_ang", 1e6)  # rad^2/s^2 for angular vel (unknown)
    big_var_ori   = rospy.get_param("~big_var_ori", 1e6)  # rad^2 for orientation (unknown)
    publish_dr_pose = rospy.get_param("~publish_dr_pose", False)

    try:
        conn = serial.Serial(port, baudrate=baudrate, timeout=1)
    except serial.serialutil.SerialException:
        rospy.logerr("ERR: cannot open serial port %s", port); return

    # Reset DR
    conn.send_break(); conn.write(b"wcr\r\n"); conn.flush()
    while not rospy.is_shutdown():
        line = conn.readline().decode(errors="ignore").strip()
        if line.startswith(("wra", "wrn")): break
    rospy.loginfo("Started DVL driver on %s @ %d baud", port, baudrate)

    while not rospy.is_shutdown():
        raw = conn.readline().decode(errors="ignore").strip()
        if not raw:
            continue

        stamp = rospy.Time.now()  # one timestamp per record

        if raw.startswith("wrz"):
            # wrz,[vx],[vy],[vz],[valid],[altitude],[fom],[cov],...
            parts = raw.replace("*","").split(",")
            try:
                vx, vy, vz = map(float, parts[1:4])
                valid = (parts[4].lower() == "y")
                fom   = float(parts[6])
                cov3  = [float(x) for x in parts[7].split(";")]   # 3x3 row-major
            except (ValueError, IndexError):
                rospy.logwarn("Malformed wrz: %s", raw); continue

            if (not valid) or (fom > fom_threshold):
                continue

            cov6 = [0.0]*36
            cov6[0:3]   = cov3[0:3]
            cov6[6:9]   = cov3[3:6]
            cov6[12:15] = cov3[6:9]
            # Angular velocity unknown → huge variances
            cov6[21] = big_var_ang
            cov6[28] = big_var_ang
            cov6[35] = big_var_ang

            msg = TwistWithCovarianceStamped()
            msg.header.stamp = stamp
            msg.header.frame_id = frame_id
            # FRD -> ENU (x: fwd->east, y: right->north, z: down->up)
            msg.twist.twist.linear.x = vx
            msg.twist.twist.linear.y = -vy
            msg.twist.twist.linear.z = -vz
            msg.twist.covariance = cov6
            pub_twist.publish(msg)
            continue

        if publish_dr_pose and raw.startswith("wrp"):
            # wrp,[tstamp],[x],[y],[z],[pos_std],[roll],[pitch],[yaw],...
            parts = raw.replace("*","").split(",")
            try:
                x, y, z = map(float, parts[2:5])
                pos_std = float(parts[5])
                roll, pitch, yaw = map(float, parts[6:9])
            except (ValueError, IndexError):
                rospy.logwarn("Malformed wrp: %s", raw); continue

            pose = PoseWithCovarianceStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = frame_id

            pose.pose.pose.position.x = x
            pose.pose.pose.position.y = -y
            pose.pose.pose.position.z = -z

            q = transformations.quaternion_from_euler(
                roll*RAD_PER_DEG, -pitch*RAD_PER_DEG, -yaw*RAD_PER_DEG)
            pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w = q

            pos_var = pos_std**2
            cov = [0.0]*36
            cov[0]  = pos_var
            cov[7]  = pos_var
            cov[14] = pos_var
            cov[21] = big_var_ori
            cov[28] = big_var_ori
            cov[35] = big_var_ori
            pose.pose.covariance = cov

            pub_pose.publish(pose)
            continue

    conn.close()

if __name__ == "__main__":
    main()
