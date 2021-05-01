#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Hydrophones signal analysis."""

import rospy
import math
import time
from auv_msgs.msg import Signals
from std_msgs.msg import Float64
from hydrophones.gccphat import estimate
from hydrophones.freq import get_frequency

FS = 1028571.4286


def analyze(msg):
    tdx = estimate(msg.quadrant_1[:-5], msg.quadrant_2[5:], FS)
    tdy = estimate(msg.quadrant_1[:-10], msg.quadrant_4[10:], FS)
    freq = get_frequency(msg.quadrant_1, FS)

    rospy.loginfo("Pingers heard frequency {}".format(freq))

    current_call = rospy.Time.now()
    time_delta = current_call - last_call

    if freq == DESIRED_FREQ and time_delta.to_sec() > 0.3:
        global last_call
        last_call = current_call

        yaw = Float64()
        yaw.data = math.atan2(tdx, tdy)

        if yaw.data > 0:
            yaw.data -= math.pi
        else:
            yaw.data += math.pi

        pub.publish(yaw)
        pubtdx.publish(tdx)
        pubtdy.publish(tdy)

if __name__ == "__main__":
    global pub
    rospy.init_node("hydrophones")
    last_call = rospy.Time.now()

    DESIRED_FREQ = rospy.get_param("hydrophones/desired_freq")

    pub    = rospy.Publisher("~heading", Float64, queue_size=1)
    pubtdx = rospy.Publisher("~tdx", Float64, queue_size=1)
    pubtdy = rospy.Publisher("~tdy", Float64, queue_size=1)
    rospy.Subscriber("/nucleo/signals", Signals, analyze, queue_size=10)
    rospy.spin()
