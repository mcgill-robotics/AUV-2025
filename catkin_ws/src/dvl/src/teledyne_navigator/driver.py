import errno
import rospy
import select
import serial
import bitstring
from teledyne_navigator.serializer import deserialize


class TeledyneNavigator(object):
    """Teledyne Navigator driver."""

    DATA_ID = 0x7D
    OUTPUT_DATA_SIZE = 88

    def __init__(self, port, baudrate, frame_id, timeout):
        """Constructs a TeledyneNavigator instance.

        Args:
            port (str): Serial port path.
            baudrate (int): Serial baudrate.
            frame_id (str): Frame ID.
            timeout (float): Serial read timeout in seconds.
        """
        self._preempted = False
        self._frame_id = frame_id
        self._conn = serial.Serial(
            port=port, baudrate=baudrate, timeout=timeout)

    def __enter__(self):
        """Initializes the DVL for first use."""
        self.open()
        return self

    def __exit__(self, *args, **kwargs):
        """Cleans up."""
        self.close()

    def open(self):
        """Opens serial connection."""
        if not self._conn.isOpen():
            self._conn.open()

        # TODO: Figure out why we need to send this twice sometimes.
        self._conn.send_break()
        self._conn.send_break()

        self._conn.flush()

    def close(self):
        """Closes serial connection."""
        self._conn.close()

    def write(self, message):
        """Writes to .

        Args:
            message (str): Message.
        """
        rospy.logdebug("Writing: %s", message)
        self._conn.write(message)

    def read(self, size=1):
        """Reads serial data.

        Args:
            size (int): Number of bytes to read, default: 1.

        Returns:
            Bytes (str).
        """
        return self._conn.read(size)

    def read_all(self):
        """Reads all serial data in the buffer.

        Returns:
            Bytes (str).
        """
        return self._conn.read_all()

    def _get_ensemble(self):
        """Gets the latest ensemble.

        Returns:
            Ensemble.
        """
        try:
            # Wait for data ID byte.
            while not self._conn.read() == chr(self.DATA_ID):
                rospy.loginfo_throttle(5.0, "Waiting for packet start byte...")
                if rospy.is_shutdown():
                    break

            # Read all ensemble data.
            packet = bitstring.BitStream("0x{:02X}".format(self.DATA_ID))
            characters = self._conn.read(size=self.OUTPUT_DATA_SIZE - 1)
            for char in characters:
                packet.append("0x{:02X}".format(ord(char)))

            return deserialize(packet, self._frame_id)
        except OSError as err:
            # Set SIGINT as KeyboardInterrupt correctly, because pyserial has
            # problems.
            # if code == errno.EINTR:
            #     raise KeyboardInterrupt()
            print(error)
            # Otherwise, reraise.
            raise

    def preempt(self):
        """Preempts a scan."""
        self._preempted = True
        self._conn.send_break()

    def set_rtc(self, date):
        """Sets the DVL's real-time clock (RTC).

        Args:
            date (Datetime): Datetime to set.
        """
        msg = "TT{}\n".format(date.strftime("%Y/%m/%d, %H:%M:%S"))
        self.write(msg)

    def scan(self, callback):
        """Starts the DVL data collection.

        This method is blocking, but calls callback at every ensemble.

        To stop a scan, simply call the preempt() method. Otherwise, the scan
        will run forever.

        Args:
            callback: Callback for feedback.
                Called with args=(TeledyneNavigator, Ensemble).
        """
        self._preempted = False

        # Set PD5 output format.
        self.write("PD5\n")

        # Set earth coordinate transformation.
        self.write("EX11111\n")

        # Start pinging.
        self.write("CS\n")

        while not self._preempted:
            # Preempt on ROS shutdown.
            if rospy.is_shutdown():
                self.preempt()
                return

            try:
                ensemble = self._get_ensemble()
                callback(self, ensemble)
            except Exception as e:
                rospy.logerr_throttle(1.0, str(e))
                continue
