#!/usr/bin/env python3

import rospy
import serial
import datetime
import math
from std_msgs.msg import Float64

def get_float(token):
    stripped = token.strip()
    return float(stripped)


def main():
    rospy.init_node("teledyne_navigator")

    x_pub = rospy.Publisher("state_x",Float64)
    y_pub = rospy.Publisher("state_y",Float64)

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

    

    # time = rospy.get_rostime()
    # date = datetime.datetime.fromtimestamp(math.ceil(time.to_time()))
    # msg = "TT{}\n".format(date.strftime("%Y/%m/%d, %H:%M:%S"))
    # conn.write(msg.encode('ascii'))
    # conn.flush()

    
    # Set PD6 output format.
    conn.write("PD6\n".encode('ascii'))
    conn.flush()
    # Set earth coordinate transformation.
    conn.write("EX11111\n".encode('ascii'))
    conn.flush()

    # Start pinging.
    conn.write("CS\n".encode('ascii'))
    conn.flush()
    
 

    # Only grabbing data we care about but this dvl can be used for more.
    # Refer to work horse manual for more info
    while conn.is_open:
        #print("-------------------------")
        line = conn.readline().decode('ascii')#print(conn.readline().decode('ascii'))
        if(line.startswith(":BD")):
            tokens = line.split(",")
            north = get_float(tokens[1])
            east = get_float(tokens[2])

            x_pub.publish(north)
            y_pub.publish(east)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        exit()
