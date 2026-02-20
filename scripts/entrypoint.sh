#!/bin/bash
set -e

# Source the base ros overlay
source /opt/ros/noetic/setup.bash

POLARIS_WS="/home/ros/polaris_ws"
STEERAI_WS="/home/ros/steerai_ws"

# If we are bind mounting workspaces, they will be owned as root (uid 0) instead of the ros user
# Bind mounts are only allowed for dev images; the ros user will have root privileges to change permissions
if [ $(stat -c "%u" "${POLARIS_WS}") -eq 0 ] || [ $(stat -c "%u" "${STEERAI_WS}") -eq 0 ]; then
    sudo chown -R ros:ros "${POLARIS_WS}" && sudo chown -R ros:ros "${STEERAI_WS}"
fi

exec "$@"



