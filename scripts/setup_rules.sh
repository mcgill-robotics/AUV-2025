#!/bin/bash

# Copy the file to the appropriate folder
cp 80-auv.rules /etc/udev/rules.d/

# Reload the rules
udevadm control --reload-rules
