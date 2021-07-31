#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Signals plotter."""

import rospy
import numpy
from nucleo.msg import Signals
from matplotlib import pyplot as plt
import time

FS = 1028571.4286


def plot(msg):
    plt.clf()

    t = numpy.arange(len(msg.quadrant_1)) / FS

    plt.subplot(4, 1, 1)
    plt.plot(t, msg.quadrant_1)
    freq = numpy.fft.rfft(msg.quadrant_1)
    plt.title(FS * (numpy.argmax(freq[10:]) + 10) / 2 / (len(freq)))

    plt.subplot(4, 1, 2)
    plt.plot(t, msg.quadrant_2)
    freq = numpy.fft.rfft(msg.quadrant_2)
    plt.title(FS * (numpy.argmax(freq[10:]) + 10) / 2 / (len(freq)))

    plt.subplot(4, 1, 3)
    plt.plot(t, msg.quadrant_3)
    freq = numpy.fft.rfft(msg.quadrant_3)
    plt.title(FS * (numpy.argmax(freq[10:]) + 10) / 2 / (len(freq)))

    plt.subplot(4, 1, 4)
    plt.plot(t, msg.quadrant_4)
    freq = numpy.fft.rfft(msg.quadrant_4)
    plt.title(FS * (numpy.argmax(freq[10:]) + 10) / 2 / (len(freq)))

    plt.draw()
    plt.pause(0.001)

if __name__ == '__main__':
    rospy.init_node("plotter")
    rospy.Subscriber("/nucleo/signals", Signals, plot, queue_size=1)
    plt.ion()
    plt.show()
    rospy.spin()