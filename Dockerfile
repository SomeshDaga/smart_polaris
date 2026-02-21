FROM ros:noetic-ros-base

ARG BUILD_DEV_IMAGE="false"

# Declare the host user/group ids
# For development inside the container via bind mounts, we want
# to ensure we are making modifications using the same user/group ids
ARG UID=1000
ARG GID=1000

# ROS 1 has reached end-of-life
# Disable warnings
ENV DISABLE_ROS1_EOL_WARNINGS=1

# Install standard tools
RUN apt update && apt install -y wget

# Install Gazebo11
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN apt update && apt install -y \
	gazebo11 \
	libgazebo11-dev \
	ros-noetic-gazebo-ros-pkgs \
	ros-noetic-gazebo-ros-control

# Install requirements listed for the polaris_gem_e2 workspace detailed in
# https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2
RUN DEBIAN_FRONTEND=noninteractive apt install -y ros-noetic-ackermann-msgs \
	ros-noetic-geometry2 \
	ros-noetic-hector-gazebo \
	ros-noetic-hector-models \
	ros-noetic-jsk-rviz-plugins \
	ros-noetic-ros-control \
	ros-noetic-ros-controllers \
	ros-noetic-velodyne-simulator \
	ros-noetic-velodyne-simulator \
        libopencv-dev

# Copy custom rosdep rules to skip bad dependencies from the polaris_gem_e2 workspace
COPY resources/rosdep-skip.yaml /etc/ros/rosdep/
RUN echo "yaml file:///etc/ros/rosdep/rosdep-skip.yaml" >> /etc/ros/rosdep/sources.list.d/10-local.list

# Add user and group inside the container corresponding to host user/group
# We name the user and group 'ros'
# This also creates the /home/ros directory with the new user/group permissions
RUN groupadd -g $GID ros && \
    useradd -m -u $UID -g $GID ros

# Copy our entrypoint script
COPY scripts/entrypoint.sh /

# Create workspace directories for our code
RUN mkdir -p /home/ros/polaris_ws/src && mkdir -p /home/ros/steer_polaris_ws/src

# Update rosdeps prior to rosdep install
RUN rosdep update --include-eol-distros

# Copy the polaris_gem_e2 packages into the polaris workspace
# Note: For development, we will bind mount directories from the host
COPY polaris_gem_e2 /home/ros/polaris_ws/src
COPY steer_polaris_gem_e2 /home/ros/steer_polaris_ws/src

# Build the workspace
WORKDIR /home/ros/polaris_ws
RUN bash -c "source /opt/ros/noetic/setup.bash && \
	rosdep install --from-paths src --ignore-src -y && \
	catkin_make"

WORKDIR /home/ros/steer_polaris_ws
RUN bash -c "source /opt/ros/noetic/setup.bash && \
	source /home/ros/polaris_ws/devel/setup.bash && \
	rosdep install --from-paths src --ignore-src -y && \
	catkin_make"

# Ensure entrypoint script is executable by our ros user
# which will be the default user during runtime
RUN chown ros:ros /entrypoint.sh && chown -R ros:ros /home/ros/polaris_ws && chown -R ros:ros /home/ros/steer_polaris_ws

# Only allow this for dev containers; disable for prod containers
RUN if [ "$BUILD_DEV_IMAGE" = "true" ]; then \
	echo "ros ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers; \
    fi

# Change to host/user group
USER ros
# Update rosdeps
RUN rosdep update --include-eol-distros
WORKDIR /home/ros

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
