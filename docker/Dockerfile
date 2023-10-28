FROM ubuntu:22.04
SHELL ["/bin/bash", "-c"]

# Settings
ENV UBUNTU_CODENAME=jammy
ENV ARCH=amd64
ENV ROS2_VERSION=humble
ENV ROS2_GPG=/usr/share/keyrings/ros-archive-keyring.gpg
ENV ROS2_PKGS_URL=http://packages.ros.org/ros2/ubuntu
ENV DEBIAN_FRONTEND=noninteractive

# Permissions
RUN apt-get update && apt-get install -y sudo
ARG USER=docker
ARG PASS=docker
ARG UID=1000
ARG GID=1000
RUN useradd -m ${USER} --uid=${UID} && echo "${USER}:${PASS}" | chpasswd
RUN adduser ${USER} sudo
ENV HOME /home/$USER

# Install ROS2 GPG key
RUN apt-get update
RUN apt-get install -qqy \
  lsb-release \
  build-essential \
  git \
  vim \
  curl \
  software-properties-common
RUN add-apt-repository universe
RUN curl \
  -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo \
  "deb [arch=$ARCH signed-by=$ROS2_GPG] $ROS2_PKGS_URL $UBUNTU_CODENAME main" \
  | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2
RUN apt-get update -y
RUN apt-get install -y ros-$ROS2_VERSION-desktop
RUN apt-get install -y python3-argcomplete python3-pip python3-future ros-dev-tools

# Install Gazebo
RUN apt-get update -y
RUN apt-get install lsb-release wget gnupg -y
RUN wget https://packages.osrfoundation.org/gazebo.gpg \
  -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update -y
RUN apt-get install -y ignition-fortress
RUN apt-get install -y libgz-plugin2-* libgz-common5-* libgz-sim7-*

# Install Oak-D DepthAI
RUN apt-get install -y ros-humble-depthai-*

# Install proto
WORKDIR $HOME
RUN git clone https://github.com/chutsu/proto \
  && cd proto && make third_party \
  && make build && make install

# Install ros-gz
RUN apt-get install -y ros-$ROS2_VERSION-actuator-msgs ros-$ROS2_VERSION-gps-msgs
WORKDIR $HOME
RUN mkdir -p $HOME/proto_ws/src \
  && cd $HOME/proto_ws/src \
  && git clone https://github.com/gazebosim/ros_gz \
  && cd ros_gz \
  && git checkout humble \
  && export GZ_VERSION=garden \
  && . /opt/ros/$ROS2_VERSION/setup.bash \
  && MAKEFLAGS="-j4" colcon build
RUN chown -R $USER:$USER $HOME/proto_ws

# Switch to $USER
USER $USER
WORKDIR $HOME
RUN echo 'export PS1="[\u@docker] \W # "' >> $HOME/.bashrc

# # Create PX4 workspace [$HOME/proto_ws]
# # Downloads the following to [$HOME/proto_ws/src]:
# # - PX4-Autopilot
# # - px4_msgs
# # - px4_ros_com
# RUN mkdir -p $HOME/proto_ws/src
# WORKDIR $HOME/proto_ws/src
# RUN git clone https://github.com/PX4/PX4-Autopilot.git
# RUN git clone https://github.com/PX4/px4_msgs.git
# RUN git clone https://github.com/PX4/px4_ros_com.git
# RUN pip3 install kconfiglib
# RUN pip3 install jinja2
# RUN pip3 install --user jsonschema
# RUN pip3 install --user -U empy pyros-genmsg setuptools
# # -- Build PX4
# WORKDIR $HOME/proto_ws/src/PX4-Autopilot
# RUN make
# # -- Build PX4 ros2 packages
# WORKDIR $HOME/proto_ws
# RUN source /opt/ros/humble/setup.bash && colcon build
# RUN . install/local_setup.bash

# Entry point script
COPY entry.sh /
ENTRYPOINT ["/entry.sh"]
