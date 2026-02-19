# Technical Assignment

## Running Application using docker

### Building

The docker image will be building using the Dockerfile provided in this repository. It contains all code and simulation dependencies, including the full gazebo simulator (including the gzclient).

The docker image can be built by running:

    ```bash
    docker build -t steerai:latest .
    ```

### Launch application

To render the gazebo simulation, we have to do the following:

1. Allow local connections to host X server

    xhost +local:docker

1. Run container with necessary environment variables and volume mounts for display access:

    docker run -it --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --net=host -it steerai:latest bash
