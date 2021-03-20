#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""Nucleo deserializer."""

import rospy
import bitstring
from serial import Serial, SerialException
from auv_msgs.msg import Signals

__author__ = "Anass Al-Wohoush"

# Define bootup header.
BOOTUP_HEADER = "[BOOTUP]"

# Define logging headers.
DEBUG_HEADER = "[DEBUG]"
FATAL_HEADER = "[FATAL]"

# Define all available headers.
headers = (
    "[DATA 1]",
    "[DATA 2]",
    "[DATA 3]",
    "[DATA 4]",
    DEBUG_HEADER,
    FATAL_HEADER,
    BOOTUP_HEADER
)


def get_header(ser):
    """Gets next header from serial buffer.

    Args:
        ser: Serial port.

    Returns:
        Header.
    """
    while ser.readable() and not rospy.is_shutdown():
        # Read until next line feed.
        header = ser.readline().strip()

        # Verify if header is valid.
        if header in headers:
            return header

        # Otherwise reset.
        if header:
            rospy.logerr("Skipped %d bytes", len(header))
        header = ""


def get_data(ser, header):
    """Gets next payload.

    Args:
        ser: Serial port.
        header: Corresponding header.

    Returns:
        Tuple of (quadrant, data).
    """
    # Get data.
    raw = ser.read(RAW_BUFFERSIZE)

    # Determine ADC.
    _, i = header.strip("]").split()
    quadrant = int(i)

    # Convert to array.
    stream = bitstring.BitStream(bytes=raw)
    data = list(
        stream.read(INT_SIZE).uintle
        for i in range(BUFFERSIZE))

    return (quadrant, data)


def iter_data(ser):
    """Iterate through data.

    Note: This is blocking.

    Args:
        ser: Serial port.

    Yields:
        Tuple of (quadrant, data).
    """
    while ser.readable() and not rospy.is_shutdown():
        header = get_header(ser)
        if "DATA" in header:
            yield get_data(ser, header)
        else:
            payload = ser.readline().strip()
            if header == DEBUG_HEADER:
                rospy.logdebug(payload)
            elif header == FATAL_HEADER:
                rospy.logfatal(payload)
            elif header == BOOTUP_HEADER:
                rospy.logwarn("Device was reset")


if __name__ == "__main__":
    rospy.init_node("nucleo", log_level=rospy.DEBUG)
    pub = rospy.Publisher("~signals", Signals, queue_size=1)

    # Get baudrate.
    baudrate = int(rospy.get_param("~baudrate", 230400))

    # Get whether 12 bit mode or not.
    TWELVE_BIT_MODE = bool(rospy.get_param("~twelve_bit_mode", False))
    rospy.loginfo("Assuming %d bit mode", 12 if TWELVE_BIT_MODE else 8)
    INT_SIZE = 16 if TWELVE_BIT_MODE else 8

    # Get buffersize.
    BUFFERSIZE = int(rospy.get_param("~buffersize", 1024))
    RAW_BUFFERSIZE = 2 * BUFFERSIZE if TWELVE_BIT_MODE else BUFFERSIZE

    # Get port name.
    port = str(rospy.get_param("~port", "/dev/nucleo"))

    # Sleep time in seconds
    sleeptime = 10.0
    rate = rospy.Rate(1.0 / sleeptime)

    while not rospy.is_shutdown():
        try:
            # Attempt to establish a serial connection to the hydrophones
            with Serial(port, baudrate=baudrate) as ser:
                rospy.loginfo("Hydrophones are connected. Serial connection successful.")
            break
        except SerialException:
            rospy.logwarn("Hydrophones are disconnected on port {}".format(port))
        # Sleep before attempting again
        rate.sleep()

    with Serial(port, baudrate=baudrate) as ser:
        rospy.loginfo("Waiting for device...")
        while not ser.readable() and not rospy.is_shutdown():
            pass
        rospy.loginfo("Found device")

        rospy.loginfo("Waiting for data...")
        signals = Signals()
        received = [0 for i in range(4)]
        previous_received_time = rospy.get_rostime()
        for quadrant, data in iter_data(ser):
            # Determine how long it has been since last packet.
            current_time = rospy.get_rostime()
            dt = current_time - previous_received_time
            previous_received_time = current_time

            rospy.logdebug("Received quadrant %d data", quadrant)

            # Reset if it has been too long, since packets might not match.
            if any(received) and dt > rospy.Duration(1):
                rospy.logwarn("Timeout %r, resetting...", dt.to_sec())
                received = [0 for i in received]

            # Verify no packet has been received multiple times.
            received[quadrant - 1] += 1
            if any(q > 1 for q in received):
                rospy.logwarn("Received inconsistent data %r", received)
                received = [0 for i in received]
                received[quadrant - 1] = 1

            # Set quadrant corresponding to packet.
            if quadrant == 1:
                signals.quadrant_1 = data
            elif quadrant == 2:
                signals.quadrant_2 = data
            elif quadrant == 3:
                signals.quadrant_3 = data
            elif quadrant == 4:
                signals.quadrant_4 = data

            # Send data if all packets were received.
            if all(q == 1 for q in received):
                pub.publish(signals)
                rospy.loginfo("Received ping")
                received = [0 for i in received]

