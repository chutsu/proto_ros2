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
ARG USERNAME
ARG USER_ID
ARG GROUP_ID
RUN addgroup --gid $GROUP_ID $USERNAME
RUN adduser --disabled-password --gecos '' --uid $USER_ID --gid $GROUP_ID $USERNAME
ADD sudoers.txt /etc/sudoers
RUN chmod 440 /etc/sudoers
ENV HOME /home/$USERNAME

# Install ROS2 GPG key
RUN apt-get update
RUN apt-get install -qqy \
  lsb-release \
  build-essential \
  git \
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

# Switch to $USERNAME
USER $USERNAME
WORKDIR $HOME
RUN echo 'export PS1="[\u@docker] \W # "' >> $HOME/.bashrc

# Create PX4 workspace [$HOME/px4_ws]
# Downloads the following to [$HOME/px4_ws/src]:
# - PX4-Autopilot
# - px4_msgs
# - px4_ros_com
RUN mkdir -p $HOME/px4_ws/src
WORKDIR $HOME/px4_ws/src
RUN git clone https://github.com/PX4/PX4-Autopilot.git
RUN git clone https://github.com/PX4/px4_msgs.git
RUN git clone https://github.com/PX4/px4_ros_com.git
RUN pip3 install kconfiglib
RUN pip3 install jinja2
RUN pip3 install --user jsonschema
RUN pip3 install --user -U empy pyros-genmsg setuptools
# -- Build PX4
WORKDIR $HOME/px4_ws/src/PX4-Autopilot
RUN make
# -- Build PX4 ros2 packages
WORKDIR $HOME/px4_ws
RUN source /opt/ros/humble/setup.bash \
  && colcon build
RUN . install/local_setup.bash

# Entry point script
# COPY entry.sh /
# ENTRYPOINT ["/entry.sh"]
