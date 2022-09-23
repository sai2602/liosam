FROM ros:melodic-ros-base-bionic

# rosdep
RUN apt-get update
RUN rosdep update

# install dependencies
RUN apt-get update && apt-get install -y \
	ros-${ROS_DISTRO}-tf  \
	netcat iputils-ping\
	vim ros-${ROS_DISTRO}-tf2 \
	ros-tf2-msgs libtf2-msgs-dev \
	ros-${ROS_DISTRO}-tf2-geometry-msgs \
	libtf2-dev libxmlrpcpp-dev \
        librosconsole-dev libactionlib-dev && \
	rm -rf /var/lib/apt/lists/*

# get source code and build REA
copy . .
RUN ./build_Release.sh
