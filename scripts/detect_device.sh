#!/bin/bash

echo "Please unplug the device you're trying to identify."
read -p "Press Enter when ready..."

# List current /dev/tty* devices
before=($(ls /dev/tty*))
echo "Captured list before plugging in the device."

echo
echo "Now plug in the device."
read -p "Press Enter when the device is plugged in..."

# List /dev/tty* devices again
after=($(ls /dev/tty*))
echo "Captured list after plugging in the device."

# Compare the two lists to find the difference
echo
echo "New device(s) detected:"
diff <(printf "%s\n" "${before[@]}" | sort) <(printf "%s\n" "${after[@]}" | sort) | grep '^>' | sed 's/^> //'