#!/usr/bin/env python3

import rospy
import serial


if __name__ == "__main__":
    rospy.init_node("teledyne_navigator")

    port = rospy.get_param("~port")
    baudrate = rospy.get_param("~baudrate")
    frame_id = rospy.get_param("~frame")
    timeout = rospy.get_param("~timeout")

    conn = serial.Serial(port)
    conn.baudrate = baudrate


    if not conn.isOpen():
        conn.open()

    conn.send_break()
    conn.send_break()
    conn.flush()

    
    # Set PD5 output format.
    conn.write("PD6\n".encode())
    # Set earth coordinate transformation.
    conn.write("EX11111\n".encode())
    # Start pinging.
    conn.write("CS\n".encode())

    conn.flush()
    
    while conn.is_open:
        print(conn.readline().decode('ascii'))