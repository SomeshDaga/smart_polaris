FROM ros:noetic-ros-base

# ROS 1 has reached end-of-life
# Disable warnings
ENV DISABLE_ROS1_EOL_WARNINGS=1

# Standard tools
RUN apt update && apt install -y wget

# Install Gazebo11
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
RUN apt update && apt install -y \
	gazebo11 \
	libgazebo11-dev \
	ros-noetic-gazebo-ros-pkgs \
	ros-noetic-gazebo-ros-control

RUN DEBIAN_FRONTEND=noninteractive apt install -y ros-noetic-ackermann-msgs \
	ros-noetic-geometry2 \
	ros-noetic-hector-gazebo \
	ros-noetic-hector-models \
	ros-noetic-jsk-rviz-plugins \
	ros-noetic-ros-control \
	ros-noetic-ros-controllers \
	ros-noetic-velodyne-simulator \
	ros-noetic-velodyne-simulator \
	# Missing upstream rosdeps due to EOL
        libopencv-dev

# Copy custom rosdep rules
COPY resources/rosdep-skip.yaml /etc/ros/rosdep/
RUN echo "yaml file:///etc/ros/rosdep/rosdep-skip.yaml" >> /etc/ros/rosdep/sources.list.d/10-local.list

# Update rosdeps
RUN rosdep update --include-eol-distros

COPY scripts/entrypoint.sh /

RUN mkdir -p /home/polaris_ws/src && mkdir -p /home/steerai_ws/src
COPY polaris_gem_e2 /home/polaris_ws/src

# Build the workspaces
WORKDIR /home/polaris_ws
RUN bash -c "source /opt/ros/noetic/setup.bash && \
	rosdep install --from-paths src --ignore-src -y && \
	catkin_make"

WORKDIR /home

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
