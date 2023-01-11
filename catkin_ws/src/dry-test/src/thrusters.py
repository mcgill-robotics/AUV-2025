#!/usr/bin/env python3

import rospy
from auv_msgs.msg import ThrusterMicroseconds
from propulsion.src.thrust_mapper import MAX_FWD_FORCE, MAX_BKWD_FORCE, force_to_microseconds

pins = {1:2, 2:3, 3:4, 4:5, 5:6, 6:7, 7:8, 8:9}
reset = [1500,1500,1500,1500,1500,1500,1500,1500]

reset_cmd = ThrusterMicroseconds(reset)
pub = rospy.Publisher('propulsion/thruster_microseconds', ThrusterMicroseconds, queue_size=10)
rospy.sleep(7)

def forwards_test(t):
    while True:
        print("- spinning at 25% max forwards force for 5s")

        cmd = reset.copy()
        cmd[t-1] = force_to_microseconds(0.25*MAX_FWD_FORCE)
        pub.publish(cmd)
        rospy.sleep(5.0)
        pub.publish(reset_cmd)

        print("1. repeat test")
        print("2. proceed")
        choice = input() 
        if choice != "1":
            break

def thruster_test(t):
    print("----------T{}----------".format(t))
    print("expected output is on pin {}".format(pins[t]))
    forwards_test(t)

def reset_thrusters():
    pub.publish(reset_cmd)
    print('Safely shutting down thrusters')

rospy.init_node("thrusters_test")
rospy.on_shutdown(reset_thrusters)

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
