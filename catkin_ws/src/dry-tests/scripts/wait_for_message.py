import rospy


class TimeoutException(Exception):
    """Timeout exception."""
    pass


class _WaitForMessage(object):

    """Subscribes to topic and waits for first message received.

    Attributes:
        received: Whether a message was received.
        message: Message received.
    """

    def __init__(self, topic, msg_type):
        """Initializes _WaitForMessage object.

        Args:
            topic: Topic name.
            msg_type: Message type.
        """
        self.received = False
        self.message = None

        # Subscribes to topic.
        self._sub = rospy.Subscriber(topic, msg_type, self._callback)

    def _callback(self, msg):
        """Subscription callback.

        Args:
            msg: Message.
        """
        self.message = msg
        self.received = True
        self.stop()

    def stop(self):
        """Unregisters from topic."""
        self._sub.unregister()


def wait_for_message(topic, msg_type, timeout):
    """Waits for first message from topic.

    Args:
        topic: Topic name.
        msg_type: Message type.
        Timeout: Timeout in seconds.

    Returns:
        First message received.

    Raises:
        TimeoutException: If no message was received in time.
    """
    start_time = rospy.get_rostime()
    waiter = _WaitForMessage(topic, msg_type)

    while not waiter.received:
        # Check if it's been too long.
        if (rospy.get_rostime() - start_time).to_sec() > timeout:
            # Unregister from topic.
            waiter.stop()
            raise TimeoutException("No message received on {}".format(topic))
            return Exception
    return waiter.message
