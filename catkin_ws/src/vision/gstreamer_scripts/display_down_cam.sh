#!/bin/bash

gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! "video/x-raw(memory:NVMM),width=4032,height=3040,framerate=30/1" ! nvvidconv ! "video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1" ! nvoverlaysink

