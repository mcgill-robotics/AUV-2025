#!/bin/bash

gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! 'video/x-raw(memory:NVMM),width=1920,height=1080,format=(string)NV12,framerate=30/1' ! nvvidconv ! 'video/x-raw(memory:NVMM), width=1280, height=720, format=(string)NV12, framerate=30/1' ! nvoverlaysink

