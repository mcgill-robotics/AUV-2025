#!/bin/bash

set -e

# Get the absolute path of the script's directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

CONTAINER_NAME="ros1-douglas-1"

# Step 1: Check if the container is already running
if docker ps --format '{{.Names}}' | grep -q "$CONTAINER_NAME"; then
    echo "[INFO] Container '$CONTAINER_NAME' is already running."
    docker exec -it "$CONTAINER_NAME" bash
    exit 0
fi

# Step 2: Ensure zed-ros-wrapper is cloned in the host workspace
ZED_WRAPPER_PATH="$SCRIPT_DIR/../../catkin_ws/src/zed-ros-wrapper"

if [ ! -d "$ZED_WRAPPER_PATH" ]; then
    echo "[INFO] Cloning zed-ros-wrapper into $ZED_WRAPPER_PATH"
    mkdir -p "$(dirname "$ZED_WRAPPER_PATH")"
    git clone --recursive https://github.com/stereolabs/zed-ros-wrapper.git "$ZED_WRAPPER_PATH"
else
    echo "[INFO] zed-ros-wrapper already exists at $ZED_WRAPPER_PATH"
fi

# Step 3: Ensure zed-ros-examples is cloned in the host workspace
ZED_EXAMPLES_PATH="$SCRIPT_DIR/../../catkin_ws/src/zed-ros-examples"

if [ ! -d "$ZED_EXAMPLES_PATH" ]; then
    echo "[INFO] Cloning zed-ros-examples into $ZED_EXAMPLES_PATH"
    mkdir -p "$(dirname "$ZED_EXAMPLES_PATH")"
    git clone --recursive https://github.com/stereolabs/zed-ros-examples.git "$ZED_EXAMPLES_PATH"
else
    echo "[INFO] zed-ros-examples already exists at $ZED_EXAMPLES_PATH"
fi

# Step 4: Bring up the container
echo "[INFO] Starting container using docker compose..."
docker compose -f "$SCRIPT_DIR/compose.yml" up -d

# Step 5: Exec into the container
echo "[INFO] Attaching to container '$CONTAINER_NAME'..."
docker exec -it "$CONTAINER_NAME" bash

