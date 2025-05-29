#!/bin/bash

# Ask for device path
read -p "Enter the device path (e.g., /dev/ttyUSB0 or /dev/ttyACM0): " DEVICE

# Check device existence
if [ ! -e "$DEVICE" ]; then
    echo "Error: Device $DEVICE does not exist."
    exit 1
fi

# Ask for rules filename
read -p "Enter the name for the .rules file (e.g., imu.rules): " RULE_FILE

# Validate filename
if [[ ! "$RULE_FILE" =~ \.rules$ ]]; then
    echo "Error: Filename must end with .rules"
    exit 1
fi

# Ask for symlink name
read -p "Enter the name for the symlink (e.g., imu, gps, sensor1): " SYMLINK_NAME

# Get udev info
UDEV_OUTPUT=$(udevadm info -a -n "$DEVICE")

# Extract fields
idVendor=$(echo "$UDEV_OUTPUT" | grep -m1 'ATTRS{idVendor}' | sed -E 's/.*=="(.*)"/\1/')
idProduct=$(echo "$UDEV_OUTPUT" | grep -m1 'ATTRS{idProduct}' | sed -E 's/.*=="(.*)"/\1/')
serial=$(echo "$UDEV_OUTPUT" | grep -m1 'ATTRS{serial}' | sed -E 's/.*=="(.*)"/\1/')

# Determine device type
KERNEL=$(basename "$DEVICE")
if [[ "$KERNEL" == ttyUSB* ]]; then
    KERNEL_PATTERN="ttyUSB[0-9]*"
elif [[ "$KERNEL" == ttyACM* ]]; then
    KERNEL_PATTERN="ttyACM[0-9]*"
else
    echo "Error: Unsupported device type $KERNEL"
    exit 1
fi

# Build the rule
RULE="SUBSYSTEM==\"tty\", KERNEL==\"$KERNEL_PATTERN\", ATTRS{idVendor}==\"$idVendor\", ATTRS{idProduct}==\"$idProduct\", ATTRS{serial}==\"$serial\", SYMLINK+=\"$SYMLINK_NAME\""

# Save the rule
echo "$RULE" > "$RULE_FILE"

# Output
echo "✅ Rule saved to $RULE_FILE:"
echo "$RULE"

