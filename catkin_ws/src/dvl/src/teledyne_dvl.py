#!/usr/bin/env python3

import rospy
import serial
import datetime
import math
from auv_msgs.msg import DvlData

def get_float(token):
    stripped = token.strip()
    return float(stripped)

def parse_BD(line):
    tokens = line.split(",")
    north = get_float(tokens[1])
    east = get_float(tokens[2])
    up = get_float(tokens[3])
    range = get_float(tokens[4])
    time = get_float(tokens[5])
    return north, east, up, range, time

def parse_TS(line):
    tokens = line.split(",")
    salinity = get_float(tokens[2])
    temp = get_float(tokens[3])
    depth = get_float(tokens[4])
    return salinity, temp, depth
    
def parse_SA(line):
    tokens = line.split(",")
    roll = get_float(tokens[1])
    pitch = get_float(tokens[2])
    heading = get_float(tokens[3])
    return roll, pitch, heading

def parse_BE(line):
    tokens = line.split(",")
    north = get_float(tokens[1])
    east = get_float(tokens[2])
    up = get_float(tokens[3])
    valid_char = tokens[3].strip()
    valid = True if valid_char == "A" else False
    return north, east, up, valid



def main():
    rospy.init_node("teledyne_navigator")

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

    data = DvlData()

    

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
    
    pub = rospy.Publisher("dvl_data",DvlData, queue_size=1)
  


    # Only grabbing data we care about but this dvl can be used for more.
    # Refer to work horse manual for more info
    while conn.is_open and not rospy.is_shutdown():
        line = conn.readline().decode('ascii')#print(conn.readline().decode('ascii'))
        if(line.startswith(":BD")):
            north, east, up, range, time = parse_BD(line)
            data.north_displacement = north
            data.east_displacement = east
            data.upward_displacement = up
            data.range_to_bottom = range
            data.time_since_good = time
            pub.publish(data)
        elif(line.startswith(":TS")):
            salinity, temp, depth = parse_TS(line)
            data.salinity = salinity
            data.temperature = temp
            data.depth = depth
            pub.publish(data)
        elif(line.startswith(":SA")):
            roll, pitch, heading = parse_SA(line)
            data.roll = roll
            data.pitch = pitch
            data.heading = heading
            pub.publish(data)
        elif(line.startswith(":BE")):
            north, east, up, valid = parse_BE(line)
            data.north_velocity = north
            data.east_velocity = east
            data.upward_velocity = up
            data.velocity_valid = valid
            pub.publish(data)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        exit()
