#!/usr/bin/env python

import math
import rospy
import datetime
from teledyne_navigator.msg import Ensemble
from teledyne_navigator import TeledyneNavigator


def callback(dvl, ensemble):
    """DVL scan data callback.

    Args:
        dvl (TeledyneNavigator): DVL instance.
        ensemble (Ensemble): DVL ensemble data.
    """
    pub.publish(ensemble)


if __name__ == "__main__":
    rospy.init_node("teledyne_navigator")

    port = rospy.get_param("~port")
    baudrate = rospy.get_param("~baudrate")
    frame_id = rospy.get_param("~frame")
    timeout = rospy.get_param("~timeout")

    pub = rospy.Publisher("~ensemble", Ensemble, queue_size=1)

    with TeledyneNavigator(port, baudrate, frame_id, timeout) as dvl:
        # Sync ROS time terribly.
        # We ceil because setting the RTC ignores any subsecond data, and seems
        # to lag by a second anyway.
        time = rospy.get_rostime()
        date = datetime.datetime.fromtimestamp(math.ceil(time.to_time()))
        dvl.set_rtc(date)

        try:
            dvl.scan(callback)
        except KeyboardInterrupt:
            dvl.preempt()
