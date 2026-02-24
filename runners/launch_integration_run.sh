#!/usr/bin/bash

set -ex

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
ROOT_DIR="${SCRIPT_DIR}/.."

# Build docker image if needed
"${SCRIPT_DIR}"/build_dev_docker.sh

xhost +local:docker
docker run -it --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --net=host --mount type=bind,src="${ROOT_DIR}"/polaris_gem_e2,dst=/home/ros/polaris_ws/src --mount type=bind,src="${ROOT_DIR}"/steer_polaris_gem_e2,dst=/home/ros/steer_polaris_ws/src --name steerai_dev -it --rm steerai:dev roslaunch gem_integration_tests integration_test.launch
