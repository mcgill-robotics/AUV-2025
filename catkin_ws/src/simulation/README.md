# Simulation


## Overview

This package contains sdf files used for (graphical) simulation of the AUV using the ignition gazebo simulator.
It also contains the models/environments that are used in the simulation.  

### License

The source code is under the [GPLv3 license](../../../LICENSE).

This package has been tested under [ROS] Noetic on Ubuntu 20.04, using Ignition Fortress.


## Installation

Follow Ignition Gazebo Fortress [installation instructions](https://gazebosim.org/docs/fortress/install_ubuntu). 


### Running

Run gazebo simulation of quali task with Tethys AUV

	cd <AUV-2020>/catkin_ws/src/simulation/worlds/
	ign gazebo quali.sdf
	

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

Run the simulation using an older version of the rendering engine (Ogre1), this can
be forced at the command line, though some assets may not be rendered correctly:

	ign gazebo quali.sdf --render-engine ogre

##### Sol. 2 - upgrade OpenGL via PPA

Alternatively, the version of OpenGL could be upgraded to be 3.3 or later. To see the current OpenGL 
version do (may need to install mesa-utils first):

	glxinfo | grep "OpenGL version"

The required OpenGL version is provided in a separate repository for stable upstream releases of X.org components, 
see [reddit post](https://www.reddit.com/r/Ubuntu/comments/8tpq05/how_can_update_my_display_driver_to_opengl_33/).
You can add this repository and upgrade:

```
sudo add-apt-repository ppa:ubuntu-x-swat/x-updates 
sudo apt-get update
sudo apt-get dist-upgrade	
```

**Note:** as stated in the message when adding the PPA, you should purge all the PPA packages prior to 
upgrading to a new release (ie. Ubuntu 20.04 -> Ubuntu 22.04).

```
If you are upgrading from one release to another with this PPA activated, please install the ppa-purge package 
and use it to downgrade everything in here beforehand. sudo ppa-purge ppa:ubuntu-x-swat/updates will do it.

```
 
##### Sol. 3 - install the up-to-date drivers for your graphics card

You can find out your GPU card model:

	lspci | grep VGA

Look up if there are any (proprietary) drivers available and instructions to install them. Often the more recent
version of these drivers will bring in a more recent version of OpenGL.
