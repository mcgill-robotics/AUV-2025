# AUV

*This project is currently under development*

Ahoy! This project contains software intended to run on a custom-built AUV to compete at RoboSub2024 on behalf of McGill Robotics.

This project is maintained by the McGill Robotics Club and was developed by its members - students of McGill University. 


## License

This software is licensed under [GPLv3](LICENSE).


## Getting Started

ROS requires Linux to run, the target operating system is Ubuntu 20.04 (LTS). For development it is sufficient to run a VM however, 
for testing it may be easier to use a native Ubuntu installation. For this, we recommend having Ubuntu on an external drive (such as a USB) 
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
