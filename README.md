# Technical Assignment

## Running Application using docker

### Building

The docker image will be building using the Dockerfile provided in this repository. It contains all code and simulation dependencies, including the full gazebo simulator (including the gzclient).

The docker image can be built by running:

    ```bash
    # For development
    docker build --build-arg BUILD_DEV_IMAGE=true -t steerai:dev .
    # For production
    docker build -t steerai:prod .
    ```

### Launch application

To render the gazebo simulation, we have to do the following:

1. Allow local connections to host X server

    xhost +local:docker

1. Run container in development mode (with bind mounted catkin workspace) and necessary environment variables and volume mounts for display access:

   docker run -it --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --net=host --mount type=bind,src=./polaris_gem_e2,dst=/home/ros/polaris_ws/src --name steerai_dev -it steerai:dev bash 
