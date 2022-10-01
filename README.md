# AUV

*This project is currently under development*

Ahoy! This project contains software intended to run on a custom built AUV to compete at RoboSub2023 on behalf of McGill Robotics. It runs using ROS and Arduino.

This project is maintained by the McGill Robotics Club and was developed by its members - students of McGill University. 


## License

This software is licensed under [GPLv3](LICENSE).


## Getting Started

ROS requires Linux to run, the target operating system is Ubuntu 20.04 (LTS). For development it is sufficient to run a VM however, 
for testing it may be easier to use a native Ubuntu installation. For this we recommend having Ubuntu on an external drive (such as a USB) 
and booting off of that to minimize the risk of screwing something up on your machine. 

ROS is a framework/bunch-o-libraries that's handy for building/running/testing robotics software, it takes care of boring-but-necessary 
things such as; providing a way to send data between running programs, and providing a common build unit (package) that allows us to 
easily integrate software other people made into our project. Make sure you are using the version of ROS compatible with your OS, for 
Ubuntu 20 this is ROS Noetic.


### Dependencies

- Ubuntu 20.04 (Focal)
- ROS Noetic
- Catkin

*+ package specific dependecies*


## Installation 

To Install ROS (desktop-full version) follow instructions on the ROS [wiki](http://wiki.ros.org/noetic/Installation/Ubuntu).

We recommend using the catkin command line tools as opposed to whatever is bundled with ROS:

    sudo apt-get install python3-catkin-tools

To get these project files onto your computer using git navigate to a desired folder and do:

    git clone https://github.com/mcgill-robotics/AUV-2020.git
  
Alternatively, if you set up your SSH keys with your Github account you can save yourself having to type your user/pass every 
time (**recommended**):

    git clone git@github.com:mcgill-robotics/AUV-2020.git


## Building ROS Packages

Before building packages, make sure that the following additional ROS packages are installed:

	sudo apt-get install ros-noetic-rosserial-arduino
	sudo apt-get install ros-noetic-pid

*To build/launch/test an individual package, see the README.md in the respective package.*

To build the currently functional packages:

	source /opt/ros/noetic/setup.bash
	cd <AUV-2020>/catkin_ws
	catkin build bringup

After build is complete, update ROS environment so that packages are 'visible':

	source ./devel/setup.bash
  
### Flashing Firmware

If you have an Arduino you can wire it up to emmulate the thrusters on the AUV (see schematic - **TODO**). Upload the 
binary file by connecting the Arduino via USB (currently hardcoded assumption USB shows up as /dev/ttyACM0) and running 
the generated make target:

    catkin build --no-deps  propulsion --make-args propulsion_embedded_thrusters-upload


## Running (on local machine)

This starts a demo mission meant to run on a local machine for testing purposes, it does not launch the state_estimation
package - you may publish to `/state` using rostopic pub (see controls package for example)

    roslaunch bringup stub.launch & 
    
You can stub the state of the AUV by publishing a `geometry_msgs/Pose` message onto `/state` topic:

	rostopic pub -r 1 /state geometry_msgs/Pose \
	"
	position:
	  x: 1.0
	  y: 2.0
	  z: -1.0
	orientation:
	  x: 0.0
	  y: 1.0
	  z: 0.0
	  w: 0.0
	" 


## Running (on AUV)
 
    roslaunch bringup bringup.launch & 
