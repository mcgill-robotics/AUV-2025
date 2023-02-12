# DVL ensemble output data.

# See 'DVL Output Data Format (PD4/PD5) Details' and 'DVL Output Data Format
# (PD5) Details' in the Navigator Technical Manual for more information.

std_msgs/Header header

# The number of beams in the DVL.
# All arrays are of this size.
int8 NUMBER_OF_BEAMS=4

# Vessel velocity w.r.t. bottom in m/s in ENU.
geometry_msgs/Vector3 bottom_velocity
float64 bottom_velocity_error
bool bottom_velocity_is_valid

# Vertical range to the bottom from each beam in m.
float64[4] beam_range_to_bottom

# Bottom status.
bool[4] beam_low_echo_amplitude
bool[4] beam_low_correlation

# Vessel velocity w.r.t. reference layer in m/s in ENU.
geometry_msgs/Vector3 reference_velocity
float64 reference_velocity_error
bool reference_velocity_is_valid

# Reference layer in m.
float64 reference_layer_start
float64 reference_layer_end

# Reference layer status.
bool reference_layer_altitude_too_shallow
bool[4] reference_layer_beam_low_correlation

# Speed of sound in m/s.
float64 speed_of_sound

# Temperature of the water at the transducer in Celsius.
float64 temperature

# Salinity in part per thousand.
float64 salinity

# Depth in m.
float64 depth

# Orientation in radians.
float64 pitch
float64 roll
float64 heading

# Distance made good over the bottom since the time of first ping in m in ENU.
geometry_msgs/Vector3 bottom_translation
float64 bottom_translation_error

# Distance made good over the reference layer since the time of first ping in m
# in ENU.
geometry_msgs/Vector3 reference_translation
float64 reference_translation_error

