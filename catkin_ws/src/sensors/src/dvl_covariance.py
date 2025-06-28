#!/usr/bin/env python3

import rospy
import serial
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped, TwistStamped, Pose
from tf import transformations
import numpy as np

RAD_PER_DEG = np.pi / 180.0

#https://docs.waterlinked.com/dvl/dvl-protocol/

def parse_dead_reckon_report(line, quat_variance):
    tokens = line.split(",")
    time_stamp = float(tokens[1])
    x = float(tokens[2])
    y = float(tokens[3])
    z = float(tokens[4])
    std = float(tokens[5])
    roll = float(tokens[6])
    pitch = float(tokens[7])
    yaw = float(tokens[8])
    status = bool(tokens[9])
    return [roll, pitch, yaw]

def main():
    rospy.init_node("waterlinked_driver")

    pub_vr = rospy.Publisher("/sensors/dvl/twist", TwistWithCovarianceStamped, queue_size=1)
    pub_dr = rospy.Publisher("/sensors/dvl/pose", PoseWithCovarianceStamped, queue_size=1)

    port = rospy.get_param("~port")
    baudrate = rospy.get_param("~baudrate")
    quat_variance = rospy.get_param("~quat_variance")

    try:
        conn = serial.Serial(port)
    except serial.serialutil.SerialException:
        rospy.logerr("ERR: /dev/dvl directory does not exist")
        rospy.sleep(5)
        exit()

    conn.timeout = 10
    # dvl's baud has been set to 115200 but its default is 9600.
    # There is a way to set the baudrate of the dvl through a command.
    conn.baudrate = baudrate

    if not conn.isOpen():
        conn.open()

    conn.send_break()
    conn.flush()

    conn.write("wcr\r\n".encode("utf-8"))
    conn.flush()

    print("Reset dead reckoning.")

    while conn.is_open and not rospy.is_shutdown():
        try:
            line = conn.readline().decode("utf-8")
            if line.startswith("wra"):
                print("INFO: DVL dead reckoning reset successful.")
                break
            elif line.startswith("wrn"):
                print("WARN: DVL dead reckoning reset failed.")
                break
        except Exception as e:
            print(e)
            break

    # Refer to work horse manual for more info
    beam_velocities = {}
    live_var = 0.0
    start = rospy.Time.now()
    eulers = []
    while conn.is_open and not rospy.is_shutdown() and rospy.Time.now() - start < rospy.Duration(30):
        try:
            line = conn.readline().decode("utf-8")
            if line.startswith("wru"):   
                parts = line.strip().split(",")
                beam_id = int(parts[1])
                vel = float(parts[2])
                # collect it
                beam_velocities[beam_id] = vel
                # once we have all 4 beams, compute variance and clear
                if len(beam_velocities) == 4:
                    vals = np.array(list(beam_velocities.values()))
                    live_var = float(np.var(vals))
                    beam_velocities.clear()
                continue
# --- dead‐reckoning pose & twist (5 Hz) ---
            if line.startswith("wrp"):
                tokens = line.split(",")
                t = float(tokens[1])
                x = float(tokens[2])
                y = float(tokens[3])
                z = float(tokens[4])
                std = float(tokens[5])
                roll = float(tokens[6])
                pitch = float(tokens[7])
                yaw = float(tokens[8])
                status = bool(tokens[9])

                #publish TwistWithCovarianceStamped (dead-reckoned speed)
                twist_msg = TwistWithCovarianceStamped()
                twist_msg.header.stamp = rospy.Time.now()
                twist_msg.header.frame_id = "dvl"
                twist_msg.twist.twist.linear.x = x
                twist_msg.twist.twist.linear.y = y
                twist_msg.twist.twist.linear.z = z

                # inject our live, per-beam variance on the diagonal:
                cov6 = [0.0]*36
                cov6[0]  = live_var  # var(x)
                cov6[7]  = live_var  # var(y)
                cov6[14] = live_var  # var(z)
                twist_msg.twist.covariance = cov6
                pub_vr.publish(twist_msg)

                pose = PoseWithCovarianceStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "auv"
                pose.pose.pose.position.x = x
                pose.pose.pose.position.y = y
                pose.pose.pose.position.z = z
                q = transformations.quaternion_from_euler(roll*RAD_PER_DEG,pitch*RAD_PER_DEG, yaw*RAD_PER_DEG)
                pose.pose.pose.orientation.x = q[0]
                pose.pose.pose.orientation.y = q[1]
                pose.pose.pose.orientation.z = q[2]
                pose.pose.pose.orientation.w = q[3]

                pos_var = std**2
                cov = [0.0]*36
                cov[0]  = pos_var
                cov[7]  = pos_var
                cov[14] = pos_var
                cov[21] = quat_variance
                cov[28] = quat_variance
                cov[35] = quat_variance
                pose.pose.covariance = cov 
                pub_dr.publish(pose)

                eulers.append(parse_dead_reckon_report(line, quat_variance))
        except Exception as e:
            print(e)
            conn.close()
            exit()
    eulers = np.array(eulers)
    eulers_cov = np.cov(eulers.T)
    print('Eulers Covariance:')
    print(eulers_cov)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        exit()