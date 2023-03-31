#!/bin/bash

gst-launch-1.0 -e nvarguscamerasrc sensor-id=0 ! "video/x-raw(memory:NVMM),width=4032,height=3040,framerate=30/1" ! nvv4l2h264enc ! h264parse ! mp4mux ! filesink location=rpi_v3_imx477_cam0.mp4
