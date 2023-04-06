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

    n_pub = rospy.Publisher("dvl_east",Float64)
    e_pub = rospy.Publisher("dvl_north",Float64)

    port = rospy.get_param("~port")
    baudrate = rospy.get_param("~baudrate")
    frame_id = rospy.get_param("~frame")
    timeout = rospy.get_param("~timeout")

    conn = serial.Serial(port)
    # dvl's baud has been set to 115200 but its default is 9600. 
    # There is a way to set the baudrate of the dvl through a command.
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

            n_pub.publish(north)
            e_pub.publish(east)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        exit()
