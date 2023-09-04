# AUV

*This project is currently under development*

Ahoy! This project contains software intended to run on a custom built AUV to compete at RoboSub2024 on behalf of McGill Robotics.

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

    git clone https://github.com/mcgill-robotics/AUV-2024.git
  
Alternatively, if you set up your SSH keys with your Github account you can save yourself having to type your user/pass every 
time (**recommended**):

    git clone git@github.com:mcgill-robotics/AUV-2024.git


## Building ROS Packages

Before building packages, make sure that the following additional ROS/python packages are installed:

	sudo apt-get install ros-noetic-rosserial-arduino
	sudo apt-get install ros-noetic-pid
	sudo apt-get install ros-noetic-joy
	sudo apt-get install ros-noetic-joy-teleop
	sudo apt-get install ros-noetic-sbg-driver
	sudo apt-get install ros-noetic-usb-cam
	sudo apt-get install ros-noetic-realsense2-camera
	pip install numpy-quaternion
	pip install ultralytics

*To build/launch/test an individual package, see the README.md in the respective package.*

To build the currently functional packages:

	source /opt/ros/noetic/setup.bash
	cd <AUV-2020>/catkin_ws
	catkin build bringup

After build is complete, update ROS environment so that packages are 'visible':

	source ./devel/setup.bash
  
  
### Flashing Firmware

If you have an Arduino you can wire it up to emmulate the thrusters on the AUV. Upload the 
binary file by connecting the Arduino via USB (currently hardcoded assumption USB shows up as /dev/ttyACM0) and running 
the generated make target:

    catkin build --no-deps  propulsion --make-args propulsion_embedded_thrusters-upload
    catkin build --no-deps  depth_sensor --make-args depth_sensor_embedded_depth_sensor-upload
    catkin build --no-deps  imu --make-args imu_embedded_imu-upload


## Running (on local machine)

This starts a demo mission that would make the AUV go forwards then backwards by publishing a `goemoetry_msgs/Wrench` message directly onto the propulsion package

    roslaunch bringup test-02-surge.launch &
    
You can see what messages are being published by running in a new terminal tab (make sure you source devel/setup.bash as environment variables will not carry over).
To see the effort:

    rostopic echo /effort
    
To see the thruster intensities:

    rostopic echo /propulsion/thruster_cmd


## Running (on AUV)
 
    roslaunch bringup bringup.launch & 
