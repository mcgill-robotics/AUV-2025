#!/bin/bash

FRAMERATE=50 # Framerate can go from 2 to 40 for 2560x1440 mode
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! "video/x-raw(memory:NVMM),width=2028,height=1080,framerate=$FRAMERATE/1" ! nvvidconv ! "video/x-raw(memory:NVMM),width=1920,height=1080,framerate=$FRAMERATE/1" ! nvoverlaysink

