#!/usr/bin/bash

set -ex

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

source "${SCRIPT_DIR}/run_variables.sh"

"${SCRIPT_DIR}/run_docker.sh" "${RUN_DOCKER_FLAGS[@]}" -- catkin test -i gem_state_manager
