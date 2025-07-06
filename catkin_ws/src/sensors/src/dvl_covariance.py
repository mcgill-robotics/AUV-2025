#!/usr/bin/env python3

import rospy
import serial
import numpy as np
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from tf import transformations

RAD_PER_DEG = np.pi / 180.0

def main():
    rospy.init_node("waterlinked_driver")

    pub_twist = rospy.Publisher("/sensors/dvl/twist", TwistWithCovarianceStamped, queue_size=1)
    pub_pose  = rospy.Publisher("/sensors/dvl/pose",  PoseWithCovarianceStamped,  queue_size=1)

    port = rospy.get_param("~port")
    baudrate = rospy.get_param("~baudrate", 115200)
    quat_var = rospy.get_param("~quat_variance", 1e-3)
    fom_threshold  = rospy.get_param("~fom_threshold", 25.0)

    try:
        conn = serial.Serial(port, baudrate=baudrate, timeout=1)
    except serial.serialutil.SerialException:
        rospy.logerr("ERR: cannot open serial port %s", port)
        return

    #reset dead-reckoning
    conn.send_break()
    conn.write(b"wcr\r\n")
    conn.flush()

    while not rospy.is_shutdown():
        line = conn.readline().decode(errors="ignore").strip()
        if line.startswith(("wra","wrn")):
            rospy.loginfo("DVL dead-reckoning reset: %s", line)
            break

    rospy.loginfo("Started DVL driver on %s @ %d baud", port, baudrate)

    # main loop
    while not rospy.is_shutdown():
        raw = conn.readline().decode(errors="ignore").strip()
        if not raw:
            continue

        # velocity + covariance
        if raw.startswith("wrz"):
            # wrz,[vx],[vy],[vz],[valid],[altitude],[fom],[cov],[t_valid],[t_tx],[time],[status]
            parts = raw.replace("*","").split(",")
            try:
                vx, vy, vz = map(float, parts[1:4])
                valid = (parts[4].lower() == "y")
                fom = float(parts[6])
                cov3 = [float(x) for x in parts[7].split(";")]
            except (ValueError, IndexError):
                rospy.logwarn("Malformed wrz: %s", raw)
                continue

            # gate on valid & FOM
            if not valid or fom > fom_threshold:
                continue

            # build 6×6 covariance
            cov6 = [0.0]*36
            cov6[0:3]    = cov3[0:3]
            cov6[6:9]    = cov3[3:6]
            cov6[12:15]  = cov3[6:9]

            msg = TwistWithCovarianceStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = "dvl"
            # FRD->ENU: flip y & z
            msg.twist.twist.linear.x = vx
            msg.twist.twist.linear.y = -vy
            msg.twist.twist.linear.z = -vz
            msg.twist.covariance = cov6

            pub_twist.publish(msg)
            continue

        #dead-reckoning pose
        if raw.startswith("wrp"):
            # wrp,[tstamp],[x],[y],[z],[pos_std],[roll],[pitch],[yaw],[status]
            parts = raw.replace("*","").split(",")
            try:
                x, y, z = map(float, parts[2:5])
                pos_std = float(parts[5])
                roll, pitch, yaw = map(float, parts[6:9])
            except (ValueError, IndexError):
                rospy.logwarn("Malformed wrp: %s", raw)
                continue

            pose = PoseWithCovarianceStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "dvl"

            # FRD->sENU position
            pose.pose.pose.position.x = x
            pose.pose.pose.position.y = -y
            pose.pose.pose.position.z = -z

            #convert RPY to quaternion; flip pitch & yaw signs for ENU
            q = transformations.quaternion_from_euler(
                roll * RAD_PER_DEG,
                -pitch * RAD_PER_DEG,
                -yaw * RAD_PER_DEG,
            )
            pose.pose.pose.orientation.x = q[0]
            pose.pose.pose.orientation.y = q[1]
            pose.pose.pose.orientation.z = q[2]
            pose.pose.pose.orientation.w = q[3]

            # covariance: position variance on diagonal, then quaternion variance
            pos_var = pos_std**2
            cov = [0.0]*36
            cov[0] = pos_var
            cov[7]  = pos_var
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
