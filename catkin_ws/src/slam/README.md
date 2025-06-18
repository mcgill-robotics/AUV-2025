Required tf Transforms
base_link → <the frame attached to sensors of incoming data>
usually a fixed value, broadcast periodically by a robot_state_publisher, or a tf static_transform_publisher.
odom → base_link
usually provided by the odometry system (e.g., the driver for the mobile base).
Provided tf Transforms
map → odom
the current odometry correction.