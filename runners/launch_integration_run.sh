#!/usr/bin/bash

set -ex

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

source "${SCRIPT_DIR}/run_variables.sh"
RUN_DOCKER_FLAGS+=( -x )

"${SCRIPT_DIR}/run_docker.sh" "${RUN_DOCKER_FLAGS[@]}" -- roslaunch gem_integration_tests integration_test.launch
