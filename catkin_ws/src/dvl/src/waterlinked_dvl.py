#!/usr/bin/env python3

import rospy
import serial
from auv_msgs.msg import DeadReckonReport, VelocityReport


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

    report = VelocityReport()
    report.vx = vx
    report.vy = vy
    report.vz = vz
    report.altitude = altitude
    report.valid = valid
    report.fom = fom
    report.covariance = covariance
    report.time_of_validity = time_of_validity
    report.time_of_transmission = time_of_transmission
    report.time = time_since_last_report
    report.status = status
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

    report = DeadReckonReport()
    report.x = x
    report.y = y
    report.z = z
    report.std = std
    report.roll = roll
    report.pitch = pitch
    report.yaw = yaw
    report.status = status
    return report
    # print(f"\nx: {x}, y: {y}, z: {z}, std: {std}, roll: {roll}, \
    #        pitch: {pitch}, yaw: {yaw}, status: {status}")
    



def main():
    rospy.init_node("waterlinked_driver")

    pub_dr = rospy.Publisher("dead_reckon_report",DeadReckonReport,queue_size=50)
    pub_vr = rospy.Publisher("velocity_report",VelocityReport,queue_size=50)

    port = rospy.get_param("~port")
    baudrate = rospy.get_param("~baudrate")

    conn = serial.Serial(port)
    # dvl's baud has been set to 115200 but its default is 9600. 
    # There is a way to set the baudrate of the dvl through a command.
    conn.baudrate = baudrate


    if not conn.isOpen():
        conn.open()

    conn.send_break()
    conn.flush()
    
    conn.write("wcr\r\n".encode("utf-8"))
    conn.flush()  
  
    print("hello world")

    # Only grabbing data we care about but this dvl can be used for more.
    # Refer to work horse manual for more info
    print(conn.is_open)
    while conn.is_open:
        # raw_data = conn.read(1).decode("utf-8")
        # print(raw_data,"")

        try:
            line = conn.readline().decode("utf-8")
            # print(line)
            if(line.startswith("wrz")):
                pub_vr.publish(parse_velocity_report(line))
            elif(line.startswith("wrp")):
                pub_dr.publish(parse_dead_reckon_report(line))
        except Exception as e:
            print(e)

         
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        exit()
