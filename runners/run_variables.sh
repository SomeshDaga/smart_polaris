#!/bin/bash

RUN_DOCKER_FLAGS=()

# Shell variable PROD=1 enables prod docker build
MODE="dev"
if [[ "${PROD:-0}" == "1" ]]; then
    MODE="prod"
fi
RUN_DOCKER_FLAGS+=( -m $MODE )

# Shell variable BIND=1 enabled bind mounting of ros workspaces
if [[ "${BIND:-0}" == "1" ]]; then
    RUN_DOCKER_FLAGS+=("-b")
fi
