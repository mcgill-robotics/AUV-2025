# AUV-2020

For more documentation and logbooks consult [Notebooks-AUV Google Drive Folder](https://drive.google.com/drive/u/0/folders/1ONxWEtG9vzzpcUIK7L2I6b7woSVX94-p)

---

## Installing

ROS works exclusively on Linux operating systems. You either need to be running a Linux distribution (recommend Ubuntu) natively as your computer's operating system or use a virtual machine. The version of the operating system needs to correspond to the version of ROS installed (see [here](http://wiki.ros.org/Distributions)). We are working with ROS Noetic which requires Ubuntu 20.04. 

_Going forward it is assumed that you are using Ubuntu, if you choose a different distro... you're on your own._ :/

It is preferrable that you install everything with the Ubuntu `apt` package manager whenever possible rather than building from source. Doing so has a number of advantages: 
- `apt` will take care of installing all the required dependencies (and making sure the versions match up)
- It becomes easier to keep track of the packages you have installed and delete them if necessary (ie. you don't have to bother with where things are installed on your computer) so if you mess up you can delete everything and start clean
- All the configuration stuff like setting the PATH environment variable is typically handled for you

Installing packages can easily be done from the command line, all it takes is knowing the name of the package you want... Prior to installing anything it is a good idea to update you local repository cache to make sure that the list of existing packages on your computer is up-to-date:

```
sudo apt-get update
```

### ROS Noetic

ROS is a collection of tools and libraries that provide functionality that is particularly useful for programming robots (it turns out a lot of this functionality has to do with facilitating the transfer of data between programs). ROS also pushes the developer to adopt a 'ROS package' architecture that promotes modularizing code based on what it does (ie. having a 'computer vision' ROS package where all the cv-related stuff resides). For development purposes it's useful to install ros with all the additional bells and whistles; the package we want is `ros-noetic-desktop-full` and can be installed following the instructions [here](http://wiki.ros.org/noetic/Installation/Ubuntu).

---

## Dependencies

### Catkin Command Line Tool

Our preferred way of building ROS packages is with the `catkin` command line tool. This tool is responsible for triggering the build processes which generates executables, and links dependencies. Ostensibly, it does the same thing as `catkin_make` (that you might encounter in tutorials) though it's a seperate tool that does not come with ROS. See here for [installation](https://catkin-tools.readthedocs.io/en/latest/installing.html).


### Python 3.x

Check if you already have the python interpreter on your system and that the version corresponds to version 3:

```
python3 --version
```

Otherwise:

```
sudo apt-get install python3
```

---

## Building

Building a project/package means taking all the source code and producing the executable files that can then be run. See the documentation included in the directory of each package for information about package dependencies and build instructions. 

To get an idea of what to build/do starting out, take a look at the [Current Packages](#current-packages) section

---

## Running

The project is currently still being developed so only _parts_ of it can be run. To run and test individual packages, see the documentation in each package directory.


---

## Current Packages

### [arduino](TODO)

### [auv_msgs](TODO)

### [controls](TODO)

### [cv](TODO)

### [planner](TODO)

### [state_estimation](TODO)

---

## Important Links

If you came across sites that were particularly helpful and that may come in handy in the future, leave them here!


**CMake**
- https://gist.github.com/baiwfg2/39881ba703e9c74e95366ed422641609
- https://samthursfield.wordpress.com/2015/11/21/cmake-dependencies-between-targets-and-files-and-custom-commands/
