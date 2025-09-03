#!/usr/bin/env python3

import rospy
import serial

def main():
    rospy.init_node("calibrate_dvl")

    # Load parameters
    port = rospy.get_param("~port")
    baudrate = rospy.get_param("~baudrate", 115200)

    try:
        conn = serial.Serial(port, baudrate=baudrate, timeout=1)
    except serial.SerialException:
        rospy.logerr("ERR: cannot open serial port %s", port)
        return

    #gyro calibration
    rospy.loginfo("Starting DVL gyro calibration...")
    conn.send_break()
    rospy.sleep(0.5)
    conn.flush()

    rospy.loginfo("Sending gyro calibration command (wcg)...")
    conn.write(b"wcg\r\n")
    conn.flush()

    start_time = rospy.Time.now()
    while conn.is_open and not rospy.is_shutdown():
        if (rospy.Time.now() - start_time).to_sec() > 15.0:
            rospy.logerr("Timeout waiting for gyro calibration response")
            break
        try:
            line = conn.readline().decode("utf-8", errors="ignore").strip()
        except Exception as e:
            rospy.logerr("Serial read error during gyro calibration: %s", e)
            break

        if line.startswith("wra"):
            rospy.loginfo("DVL gyro calibration successful.")
            break
        elif line.startswith("wrn"):
            rospy.logwarn("DVL gyro calibration failed.")
            break

    conn.close()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
