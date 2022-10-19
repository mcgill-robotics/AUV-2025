# Simulation


## Overview

This package contains sdf files used for (graphical) simulation of the AUV using the ignition gazebo simulator.
It also contains the models/environments that are used in the simulation.  

The propulsion package has been tested under ROS Noetic for Ubuntu 20.04.

### License

The source code is released under a GPLv3 license.

## Installation

Follow Ignition Gazebo Fortress [installation instructions](https://gazebosim.org/docs/fortress/install_ubuntu). 

Install the `ros_ign_bridge` package from source

	cd <AUV-2020>/catkin_ws
	git submodule update
	export IGNITION_VERSION=fortress
	rosdep install -r --from-paths src -i -y --rosdistro noetic
	source /opt/ros/noetic/setup.bash
	catkin build simulation


### Running

Run gazebo simulation of quali task with Tethys AUV

	cd <AUV-2020>/catkin_ws/src/simulation/worlds/
	ign gazebo quali.sdf
	
To interface with gazebo using ROS:

	roscore &
	rosrun simulation test_tethys.py &
	rosrun ros_ign_bridge parameter_bridge \ 
	/model/tethys/joint/propeller_joint/cmd_pos@std_msgs/Float64@ignition.msgs.Double &


### Usage

In the GUI, you should see a pole, gate and AUV models. Click the play button in the bottom left to 
run/pause the simulation. In a new tab you can publish commands to the model through the command line.

Control the propellor of the Tethys to make it go forward/backwards (you should see the propellor spin):

	ign topic -t /model/tethys/joint/propeller_joint/cmd_pos -m ignition.msgs.Double -p 'data: -31'

Control the vertical fins to affect the yaw of the Tethys while moving. 

	ign topic -t /model/tethys/joint/vertical_fins_joint/0/cmd_pos -m ignition.msgs.Double -p 'data: -0.17'

Control the horizontal fins to affect the pitch of the Tethys while moving. 

	ign topic -t /model/tethys/joint/horizontal_fins_joint/0/cmd_pos -m ignition.msgs.Double -p 'data: -0.20'


### Known Issues

#### Ogre2 Requires OpenGL > 3.3

After fresh installation, trying to run gazebo (as below) may result in an error containing:

```
Unable to create the rendering window: OGRE EXCEPTION(3:RenderingAPIException): OpenGL 3.3 is not supported.
```

It seems Ignition Fortress is using Ogre2 (rendering engine) which requires OpenGL > 3.3, 
see [forum post](https://answers.gazebosim.org/question/27597/ignition-crashes-directly-after-start/). 

##### Sol. 1 - Use Ogre1 as render engine

Run the simulation using an older version of the rendering engine (Ogre1) that uses an older version of 
the OpenGL standard, this can be forced at the command line, though some assets may not be rendered correctly:

	ign gazebo quali.sdf --render-engine ogre


##### Sol. 2 - install the up-to-date drivers for your graphics card via PPA

OpenGL is not a program, it is a standard that the CPU uses to give graphics-related commands to the GPU. 
We require the implementation of the OpenGL 3.3 standard - the actual implementation of this standard is 
likely done by the manufacturers of our GPU card as part of their operating-system compatible GPU drivers. 
To see the current OpenGL version supported by your GPU drivers (may need to install mesa-utils first):

	glxinfo | grep "OpenGL version"

To find out your GPU card model:

	sudo lshw -c video

Do some research to try and find drivers for your graphics card, often the latest drivers using OpenGL > 3.3 are 
provided via PPA - to fix this issue on the MEDN-WS1 desktop, the following was done:

```
sudo add-apt-repository ppa:kisak/kisak-mesa
sudo apt-get dist-upgrade	
```

**Note:** as stated in the message when adding the PPA, you should install ppa-purge and downgrade all PPA packages 
prior to upgrading to a new release (ie. Ubuntu 20.04 -> Ubuntu 22.04). Example for the PPA used on MEDN-WS1:

```
sudo ppa-purge -d focal ppa:kisak/kisak-mesa
```
