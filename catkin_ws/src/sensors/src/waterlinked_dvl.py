#!/usr/bin/env python3

import rospy
import serial
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from tf import transformations
import numpy as np

RAD_PER_DEG = np.pi / 180.0

def parse_velocity_report(line):
    tokens = line.split(",")
    vx = float(tokens[1])
    vy = float(tokens[2])
    vz = float(tokens[3])
    valid_string = tokens[4]
    if valid_string == "y":
        valid = True
    else:
        valid = False
    altitude = float(tokens[5])
    fom = float(tokens[6])
    covariance = [float(x) for x in tokens[7].split(";")]
    time_of_validity = float(tokens[8])
    time_of_transmission = float(tokens[9])
    time_since_last_report = float(tokens[10])
    status = bool(tokens[11])

    report = TwistWithCovarianceStamped()
    report.twist.twist.linear.x = vx
    report.twist.twist.linear.y = -vy
    report.twist.twist.linear.z = -vz
    # report.twist.covariance = covariance
    for i in range(3):
        for j in range(3):
            report.twist.covariance[i * 6 + j] = covariance[i * 3 + j]
    report.header.frame_id = "dvl"
    report.header.stamp = rospy.Time.now()

    return report
    # print(f"\nvx: {vx}, vy: {vy}, vz: {vz}, valid: {valid}, altitude: {altitude}, \
    #        fom: {fom}, covariance: {covariance}, time valid: {time_of_validity}, time_of_transmission: {time_of_transmission}, \
    #         time_since_last_report: {time_since_last_report}, state: {state}")


def parse_dead_reckon_report(line):
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

    report = PoseWithCovarianceStamped()
    report.pose.pose.position.x = 0
    report.pose.pose.position.y = 0
    report.pose.pose.position.z = 0

    quaternion = transformations.quaternion_from_euler(roll * RAD_PER_DEG, pitch * RAD_PER_DEG, yaw * RAD_PER_DEG)
    report.pose.pose.orientation.x = quaternion[0]
    report.pose.pose.orientation.y = quaternion[1]
    report.pose.pose.orientation.z = quaternion[2]
    report.pose.pose.orientation.w = quaternion[3]
    report.pose.covariance = [0.0] * 36
    for i in range(3,6):
        report.pose.covariance[i * 6 + i] = std
    report.header.frame_id = "dvl"
    report.header.stamp = rospy.Time.now()
    return report

def main():
    rospy.init_node("waterlinked_driver")

    pub_vr = rospy.Publisher("/sensors/dvl/twist", TwistWithCovarianceStamped, queue_size=1)
    pub_dr = rospy.Publisher("/sensors/dvl/pose", PoseWithCovarianceStamped, queue_size=1)

    port = rospy.get_param("~port")
    baudrate = rospy.get_param("~baudrate")


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

    # Only grabbing data we care about but this dvl can be used for more.
    # Refer to work horse manual for more info
    while conn.is_open and not rospy.is_shutdown():
        try:
            line = conn.readline().decode("utf-8")
            # print(line)
            if line.startswith("wrz"):
                pub_vr.publish(parse_velocity_report(line))
            elif line.startswith("wrp"):
                pub_dr.publish(parse_dead_reckon_report(line))
        except Exception as e:
            print(e)
            conn.close()
            exit()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        exit()
