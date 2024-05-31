#!/usr/bin/env python3

import rospy
from auv_msgs.msg import ThrusterMicroseconds

force_amt = 0.0025  # 0.5%

# copied from thrust_mapper

# forces produced by T200 thruster at 14V (N)
MAX_FWD_FORCE = 4.52 * 9.81
MAX_BKWD_FORCE = -3.52 * 9.81


def negativeForceCurve(force):
    return (
        1.4701043632380542 * (10**3)
        + force * 2.3999978362959104 * (10**2)
        + (force**2) * 2.5705773429064880 * (10**2)
        + (force**3) * 3.1133962497995367 * (10**2)
        + (force**4) * 2.1943237103469241 * (10**2)
        + (force**5) * 8.4596303821198617 * (10**1)
        + (force**6) * 1.6655229499580056 * (10**1)
        + (force**7) * 1.3116834437073399
    )


def positiveForceCurve(force):
    return (
        1.5299083405100268 * (10**3)
        + force * 1.9317247519327023 * (10**2)
        + (force**2) * -1.6227874418158476 * (10**2)
        + (force**3) * 1.4980771349508325 * (10**2)
        + (force**4) * -8.0478019175136623 * (10**1)
        + (force**5) * 2.3661746039755371 * (10**1)
        + (force**6) * -3.5559291204780612
        + (force**7) * 2.1398707591286295 * (10**-1)
    )


def force_to_microseconds(force):
    # cap our input force at maximum fwd/bkwd speeds
    force = min(max(force, MAX_BKWD_FORCE), MAX_FWD_FORCE)
    # two different curves (negative and positive forces)
    if force > 0.0:
        return int(positiveForceCurve(force / 9.81))
    elif force < 0.0:
        return int(negativeForceCurve(force / 9.81))
    else:
        return 1500  # middle value is 1500


# TODO: update if necessary
pins = {1: 4, 2: 5, 3: 2, 4: 6, 5: 8, 6: 3, 7: 7, 8: 1}
reset = [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500]

reset_cmd = ThrusterMicroseconds(reset)
pub = rospy.Publisher("/propulsion/microseconds", ThrusterMicroseconds, queue_size=1)
rospy.sleep(7)


def forwards_test(t):
    print("----------T{}----------".format(t))
    print("expected output is on PWM {}".format(pins[t]))
    while not rospy.is_shutdown():
        print("- spinning at " + str(100 * force_amt) + "% max forwards force for 5s")

        cmd = reset.copy()
        cmd[t - 1] = force_to_microseconds(force_amt * MAX_FWD_FORCE)
        pub.publish(cmd)
        rospy.sleep(5.0)
        pub.publish(reset_cmd)

        print("1. repeat test")
        print("2. proceed")
        choice = input()
        if choice != "1":
            break


def simultaneous_forwards_test():
    print_str = "|"
    for t in range(8):
        print_str += " T{} - PWM {} |".format(t + 1, pins[t + 1])
    print(print_str)
    while not rospy.is_shutdown():
        print("- spinning at " + str(100 * force_amt) + "% max forwards force for 5s")

        cmd = [force_to_microseconds(force_amt * MAX_FWD_FORCE)] * 8
        pub.publish(cmd)
        rospy.sleep(5.0)
        pub.publish(reset_cmd)

        print("1. repeat test")
        print("2. proceed")
        choice = input()
        if choice != "1":
            break


def reset_thrusters():
    pub.publish(reset_cmd)
    print("Safely shutting down thrusters")


def re_arm():
    rospy.sleep(1)
    msg1 = ThrusterMicroseconds([1500]*8)
    msg2 = ThrusterMicroseconds([1540]*8)

    pub.publish(msg1)
    rospy.sleep(0.5)
    pub.publish(msg2)
    rospy.sleep(0.5)
    pub.publish(msg1)

rospy.init_node("thrusters_test")
rospy.on_shutdown(reset_thrusters)

while not rospy.is_shutdown():
    print("========== Thrusters Test ==========")
    print(
        "refer to images in dry-test/images for labelled diagrams of thrusters on the AUV"
    )
    print("1. test thrusters one by one")
    print("2. select thruster to test")
    print("3. test thrusters simultaneously")
    print("4. re-arm")
    print("5. exit")
    choice = input()

    if choice == "1":
        forwards_test(1)
        forwards_test(2)
        forwards_test(3)
        forwards_test(4)
        forwards_test(5)
        forwards_test(6)
        forwards_test(7)
        forwards_test(8)
    elif choice == "2":
        choice = int(input("select thruster to test (1-8): "))
        if 1 <= choice and choice <= 8:
            forwards_test(choice)
    elif choice == "3":
        simultaneous_forwards_test()
    elif choice == "4":
        re_arm()
    else:
        break
