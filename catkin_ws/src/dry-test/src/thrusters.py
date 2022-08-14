#!/usr/bin/env python3

import rospy
from auv_msgs.msg import ThrusterCommand

pins = {1:2, 2:3, 3:4, 4:5, 5:6, 6:7, 7:8, 8:9}
reset = [0,0,0,0,0,0,0,0]
reset_cmd = ThrusterCommand(reset)
pub = rospy.Publisher('propulsion/thruster_cmd', ThrusterCommand, queue_size=10)
rospy.sleep(7)

def forwards_test(t):
    while True:
        print("- spinning at 5% power forwards for 5s")

        cmd = reset.copy()
        cmd[t-1] = 0.05
        pub.publish(cmd)
        rospy.sleep(5.0)
        pub.publish(reset_cmd)

        print("1. repeat test")
        print("2. proceed")
        choice = input() 
        if choice == "2":
            break

def thruster_test(t):
    print("----------T{}----------".format(t))
    print("expected output is on pin {}".format(pins[t]))
    forwards_test(t)


rospy.init_node("thrusters_test")
#pub.publish(reset_cmd)
#rospy.sleep(7)
while True:
    print("========== Thrusters Test ==========")
    print("refer to images in dry-test/images for labelled diagrams of thrusters on the AUV")
    print("1. test all thrusters")
    print("2. select thruster to test")
    print("3. exit")
    choice = input()

    if choice == "1":
        thruster_test(1)
        thruster_test(2)
        thruster_test(3)
        thruster_test(4)
        thruster_test(5)
        thruster_test(6)
        thruster_test(7)
        thruster_test(8)

    elif choice == "2":
        choice = int(input("select thruster to test (1-8): "))
        if 1 <= choice and choice <=8:
            thruster_test(choice)

    else:
        break
