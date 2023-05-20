# State estimation

## Overview

The state estimation package is responsible for reading in sensor messages, uses state estimation algorithms
to estimate the pose of the auv, then publishes the state in the topics pose, state_x, state_y, state_z,
state_theta_x, state_theta_y, and state_theta_z.

The state estimation algorithm used is the Kalman filter. Our implementation of the Kalman filter only does filter on the x and y axes. We chose not to do this on the other axes because the sensor measurements because the measurements are more reliable. For example, the depth sensor gives an unbiased reading of the depth. The same applies for the imu along the rotational axes. However, to estimate the x and y positions, we use the dvl's velocity data to estimate x and y with dead reckoning. While this is yields good results, the errors in velocity estimates are integrated, resulting the x and y estimates that drift the longer the auv is run. To minimize this drifting, the Kalman filter combines the imu acceleration measurements to get more precise estimates of the velocity.

Currently the Kalman filter is not implemented in the state estimation package.

In the state estimation package, we define the global frame as North East down, and the world frame as North East Up. We are aware that this naming is confusing and will likely be changed in the future. The imu returns orientation estimates in the global frame while the rest of the code base works in the world frame.