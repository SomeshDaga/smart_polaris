#!/usr/bin/bash

set -ex

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
ROOT_DIR="${SCRIPT_DIR}/.."

# Build docker image if needed
"${SCRIPT_DIR}"/build_dev_docker.sh

docker run -it --name steerai_dev -it --rm steerai:dev bash -c "catkin test gem_state_manager" 
