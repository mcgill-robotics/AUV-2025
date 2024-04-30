# Propulsion


## Overview


The propulsion package is responsible for moving the AUV and provides a hardware-agnostic interface.

The propulsion package has been tested under ROS Noetic for Ubuntu 20.04.

### License

The source code is released under a GPLv3 license.

## Package Interface

### Published Topics

| Topic | Message | Description |
| ------ | ------- | ---------- |
| `batt1_voltage` | `std_msgs/Float32` | Battery 1 voltage |
| `batt2_voltage` | `std_msgs/Float32` | Battery 2 voltage |
| `thrust1_current` | `std_msgs/Float32` | Thruster 1 current |
| `thrust2_current` | `std_msgs/Float32` | Thruster 2 current |
| `thrust3_current` | `std_msgs/Float32` | Thruster 3 current |
| `thrust4_current` | `std_msgs/Float32` | Thruster 4 current |
| `thrust5_current` | `std_msgs/Float32` | Thruster 5 current |
| `thrust6_current` | `std_msgs/Float32` | Thruster 6 current |
| `thrust7_current` | `std_msgs/Float32` | Thruster 7 current |
| `thrust8_current` | `std_msgs/Float32` | Thruster 8 current |

### Subscribed Topics

| Topic | Message | description |
| ------ | ------- | ---------- |
| `/effort` | `geometry_msgs/Wrench` | Force and torque, relative to the robot's frame of reference to be applied at a given moment |
| `/propulsion/microseconds` | `auv_msgs/ThrusterMicroseconds` | Thruster control commands |


## Installation

### Dependencies

- `catkin`
- `geometry_msgs`
- `propulsion_msgs`
- `rosserial_arduino`
- `rosserial_client`

### Building

	source /opt/ros/noetic/setup.bash
	cd AUV-2024/catkin_ws/src
	catkin build propulsion

After build is complete, make the packages visible to ROS

	source ../devel/setup.bash

### Running

Compile teensy (for debugging embedded code)

	catkin build --no-deps  propulsion --make-args compile_teensy

Flash teensy

	catkin build --no-deps  propulsion --make-args upload_teensy

Launch all package nodes

	roslaunch propulsion propulsion.launch
	
### Usage

Publishing a `geometry_msgs/Wrench` message onto `/effort` topic:
	
	rostopic pub -1 /effort geometry_msgs/Wrench "{force: {x: 1.0, y: 0.0, z: -0.5}, torque: {x: 1.0, y: -0.5, z: -2.0}}"

## Installation Steps for Arduino CLI (required to compile and upload to teensy)

1. **Download**: 
   - Download the latest version of the Arduino CLI from the [Arduino CLI GitHub releases page](https://github.com/arduino/arduino-cli/releases).

2. **Extract Archive**: 
   - Extract the downloaded archive using the following command:
     ```
     tar xf arduino-cli_*.tar.gz
     ```

3. **Move to Bin Directory**: 
   - Move the extracted `arduino-cli` executable to the `/usr/local/bin` directory using sudo:
     ```
     sudo mv arduino-cli /usr/local/bin
     ```

4. **Check Version**: 
   - Confirm the installation by checking the version of Arduino CLI:
     ```
     arduino-cli version
     ```

5. **Configuration**: 
   - Open or create the `arduino-cli.yaml` file. This file is usually located in your home directory `~/.arduino15/arduino-cli.yaml` on Unix-like systems.

6. **Add Additional URLs**: 
   - Add the following lines to the `arduino-cli.yaml` file:
     ```yaml
     board_manager:
       additional_urls: [https://www.pjrc.com/teensy/package_teensy_index.json]
     ```

7. **Update Index**: 
   - Update the core index using the following command:
     ```
     arduino-cli core update-index
     ```

8. **List Available Platforms**: 
   - List the available platforms to confirm the addition of Teensy:
     ```
     arduino-cli core search
     ```
     Teensy:avr should be displayed.

9. **Install Teensy Platform**: 
   - Install the Teensy platform using the following command:
     ```
     arduino-cli core install teensy:avr
     ```
