#!/usr/bin/bash
set -e

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

IMAGE_NAME="steerai:dev"
docker build --build-arg BUILD_DEV_IMAGE=true -t ${IMAGE_NAME} ${SCRIPT_DIR}/..
#if [[ -z "$(docker images -q ${IMAGE_NAME} 2> /dev/null)" ]]; then
#    echo "Docker image '${IMAGE_NAME}' not found. Building..."
#    docker build --build-arg BUILD_DEV_IMAGE=true -t ${IMAGE_NAME} ${SCRIPT_DIR}/..
#    echo "Build complete."
#else
#    echo "Docker image '${IMAGE_NAME}' already exists."
#fi
