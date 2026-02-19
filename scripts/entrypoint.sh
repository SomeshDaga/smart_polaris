#!/bin/bash

set -e

# Source the base ros overlay
source /opt/ros/noetic/setup.bash

POLARIS_WS="/home/polaris_ws"
STEERAI_WS="/home/steerai_ws"

source "${POLARIS_WS}/devel/setup.bash"
# TODO(someshdaga): Source the steerai_ws

exec "$@"



