#!/usr/bin/env bash

set -e

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
ROOT_DIR="${SCRIPT_DIR}/.."

# Defaults
MODE="dev"
ENABLE_X11=false
ENABLE_BIND=false

IMAGE_DEV="polaris:dev"
IMAGE_PROD="polaris:prod"

POLARIS_WORKSPACE="$ROOT_DIR/polaris_gem_e2"
SMART_POLARIS_WORKSPACE="$ROOT_DIR/smart_polaris_gem_e2"

usage() {
    echo "Usage:"
    echo "  $0 [OPTIONS] -- <command>"
    echo ""
    echo "Options:"
    echo "  -m, --mode  [dev|prod]"
    echo "  -x, --x11   Enable X11 forwarding"
    echo "  -b, --bind  Enable bind mounts"
    echo ""
    echo "Example:"
    echo "  $0 -m dev -x -- catkin build"
    exit 1
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case "$1" in
        -m|--mode)
            MODE="$2"
            shift 2
            ;;
        -x|--x11)
            ENABLE_X11=true
            shift
            ;;
        -b|--bind)
            ENABLE_BIND=true
            shift
            ;;
        --)
            shift
            break
            ;;
        -*)
            echo "Unknown option $1"
            usage
            ;;
        *)
            # first non-option → command starts
            break
            ;;
    esac
done

# Remaining args = container command
if [[ $# -eq 0 ]]; then
    CMD=(/bin/bash)
else
    CMD=("$@")
fi

# Check image is one of prod or dev
DOCKER_BUILD_ARGS=()
case "$MODE" in
    dev)
        IMAGE="$IMAGE_DEV"
	DOCKER_BUILD_ARGS+=(
	    --build-arg BUILD_DEV_IMAGE="true"
	)	
	;;
    prod)
        IMAGE="$IMAGE_PROD"
	;;
    *)
        echo "Invalid mode: $MODE"
        exit 1
        ;;
esac

CONTAINER_NAME="polaris_$MODE"

# Build image
echo "Building $IMAGE"
docker build "${DOCKER_BUILD_ARGS[@]}" -t "$IMAGE" "${ROOT_DIR}"

DOCKER_RUN_ARGS=(
    --rm
    -it
    --name "$CONTAINER_NAME"
)

# Bind mounts
if $ENABLE_BIND; then
    DOCKER_RUN_ARGS+=(
        --mount type=bind,src="$POLARIS_WORKSPACE",dst="/home/ros/polaris_ws/src"
        --mount type=bind,src="$SMART_POLARIS_WORKSPACE",dst="/home/ros/smart_polaris_ws/src"
    )
fi

# X11 Forwarding
if $ENABLE_X11; then
    xhost +local:docker >/dev/null 2>&1 || true

    DOCKER_RUN_ARGS+=(
        -e DISPLAY=$DISPLAY
        -v /tmp/.X11-unix:/tmp/.X11-unix
        -e QT_X11_NO_MITSHM=1
    )
fi

# Run container
echo "Running: ${CMD[*]}"
docker run \
    "${DOCKER_RUN_ARGS[@]}" \
    "$IMAGE" \
    "${CMD[@]}"
