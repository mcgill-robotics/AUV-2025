import rospy
import bitstring
import numpy as np
from teledyne_navigator.msg import Ensemble


def validate_checksum(bytestring, expected_checksum):
    """Validates a checksum.

    Args:
        bytestring (str): String of bytes to run checksum on.
        expected_checksum (int): Expected checksum.

    Raises:
        ValueError: If the checksum doesn't match.
    """
    checksum = sum(map(ord, bytestring)) % 65535
    if checksum != expected_checksum:
        raise ValueError("Checksum mismatch: ignoring message")


def deserialize(buff, frame_id):
    """Deserializes DVL ensemble data in the PD5 binary formats.

    Args:
        buff (BitString): Buffer.
        frame_id (str): Frame ID.

    Returns:
        Ensemble.

    Raises:
        ValueError: If the data is invalid or could not be deserialized.
    """
    data = Ensemble()
    data.header.frame_id = frame_id

    # TODO: Replace this with the ping time from the packet once syncing is
    # more proper.
    data.header.stamp = rospy.get_rostime()

    # Maintain start position of the buffer for checksum computation.
    start = buff.bytepos

    # DVL data ID, expected to be 0x7D.
    data_id = buff.read("uint:8")
    if data_id != 0x7D:
        raise ValueError("Unexpected data ID: {}".format(data_id))

    # Whether the format is PD4 or PD5.
    #   0: PD4 format.
    #   1: PD5 format.
    data_structure = buff.read("uint:8")
    if data_structure != 1:
        raise ValueError("Unexpected data format: {}".format(data_structure))

    # Number of bytes sent excluding the checksum.
    _ = buff.read("uintle:16")

    # System configuration.
    _ = buff.read("uint:8")

    # Velocity of the vessel w.r.t. the bottom.
    # Convert from mm/s to m/s.
    x_vel = buff.read("intle:16")
    data.bottom_velocity_is_valid = x_vel != -32768
    data.bottom_velocity.x = x_vel / 1e3
    data.bottom_velocity.y = buff.read("intle:16") / 1e3
    data.bottom_velocity.z = buff.read("intle:16") / 1e3
    data.bottom_velocity_error = buff.read("intle:16") / 1e3

    # Vertical range to bottom from each beam.
    # Convert from cm to m.
    data.beam_range_to_bottom = [
        buff.read("uintle:16") / 1e2 for i in range(Ensemble.NUMBER_OF_BEAMS)
    ]

    # Status of the bottom-referenced correlation and echo amplitude data.
    bottom_status = reversed(
        [buff.readlist("bool, bool") for _ in range(Ensemble.NUMBER_OF_BEAMS)])
    data.beam_low_echo_amplitude, data.beam_low_correlation = zip(
        *bottom_status)

    # Velocity of the vessel w.r.t. the water-mass reference layer.
    # Convert from mm/s to m/s.
    x_vel = buff.read("intle:16")
    data.reference_velocity_is_valid = x_vel != -32768
    data.reference_velocity.x = x_vel / 1e3
    data.reference_velocity.y = buff.read("intle:16") / 1e3
    data.reference_velocity.z = buff.read("intle:16") / 1e3
    data.reference_velocity_error = buff.read("intle:16") / 1e3

    # Water-mass reference layer depths.
    # Convert from dm to m.
    data.reference_layer_start = buff.read("uintle:16") / 1e1
    data.reference_layer_end = buff.read("uintle:16") / 1e1

    # Reference status.
    _ = buff.read("uint:3")  # Unused.
    data.reference_layer_altitude_too_shallow = buff.read("bool")
    data.reference_layer_beam_low_correlation = reversed(
        [buff.read("bool") for _ in range(Ensemble.NUMBER_OF_BEAMS)])

    # Time of first ping of the current ensemble.
    # TODO: Use this instead of ROS time.
    _ = buff.read("uint:8")  # Hour
    _ = buff.read("uint:8")  # Minute
    _ = buff.read("uint:8")  # Second
    _ = buff.read("uint:8")  # Hundredth

    # Result of built-in tests.
    # 0 means successful.
    _ = buff.read("uintle:16")

    # Speed of sound in m/s.
    data.speed_of_sound = buff.read("uintle:16")

    # Temperature of the water at the transducer head.
    # Convert from 0.01 C to 1 C.
    data.temperature = buff.read("intle:16") / 1e2

    # Salinity in part-per-thousand.
    data.salinity = buff.read("uint:8")

    # Depth.
    # Convert from dm to m.
    data.depth = buff.read("uintle:16") / 1e1

    # Orientation.
    # Convert from 0.01 degrees to degrees.
    data.pitch = np.radians(buff.read("intle:16") / 1e2)
    data.roll = np.radians(buff.read("intle:16") / 1e2)
    data.heading = np.radians(buff.read("uintle:16") / 1e2)

    # Distance Made Good (DMG) over the bottom since the first ping.
    # Convert from dm to m.
    data.bottom_translation.x = buff.read("intle:32") / 1e1
    data.bottom_translation.y = buff.read("intle:32") / 1e1
    data.bottom_translation.z = buff.read("intle:32") / 1e1
    data.bottom_translation_error = buff.read("intle:32") / 1e1

    # Distance Made Good (DMG) over the water-mass reference layer since the
    # first ping.
    # Convert from dm to m.
    data.reference_translation.x = buff.read("intle:32") / 1e1
    data.reference_translation.y = buff.read("intle:32") / 1e1
    data.reference_translation.z = buff.read("intle:32") / 1e1
    data.reference_translation_error = buff.read("intle:32") / 1e1

    # Validate checksum.
    end = buff.bytepos
    expected_checksum = buff.read("uintle:16")
    bytestring = buff.bytes[start:end]
    validate_checksum(bytestring, expected_checksum)

    return data
