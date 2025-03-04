# Propulsion Package

## Setup

Before running any commands, make sure to build the package:
```bash
colcon build
```
Then source the setup file:
```bash
source install/setup.bash
```

## Launching Propulsion

To launch the propulsion system, use:
```bash
ros2 launch propulsion propulsion_launch.py
```

## Dry Test

For a dry test, run:
```bash
ros2 run propulsion drytest
```

## Keyboard Control

To enable keyboard control, the thruster mapper node must be launched separately:
```bash
ros2 run propulsion thrust_mapper
```
Alternatively, you can launch everything with:
```bash
ros2 launch propulsion propulsion_launch.py
```

## TODO

The following improvements need to be implemented and documented:
1. Integrate Micro-ROS into propulsion as a node and test it.
2. Add `auv_msgs` and `microros_agent` as dependencies.
3. Convert `/propulsion/microseconds` to an `Int16MultiArray`.
4. Ensure Micro-ROS setup is reconfigured when the repository is compiled.
5. Clean up the propulsion package.
