#!/bin/bash
set -e

# Source the base ros overlay
source /opt/ros/noetic/setup.bash

POLARIS_WS="/home/ros/polaris_ws"
# If we are bind mounting the workspace, it will be owned as root (uid 0) instead of the ros user
# Bind mounts are only meant to be used dev images
# The ros user will have root privileges to change permissions to the host user/group in dev mode
if [ $(stat -c "%u" "${POLARIS_WS}") -eq 0 ]; then
    sudo chown -R ros:ros "${POLARIS_WS}" && sudo chown -R ros:ros "${STEERAI_WS}"
fi

source "${POLARIS_WS}/devel/setup.bash"

exec "$@"



