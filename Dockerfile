FROM ros:humble-ros-base

ARG USERNAME=cvdoc
ARG USER_UID=1000
ARG USER_GID=$USER_UID

SHELL ["/bin/bash", "-c"]
ENV SHELL=/bin/bash

RUN rm -f /etc/apt/sources.list && curl -sSL http://mirrors.pku.edu.cn/repoconfig/ubuntu22.04/sources.list -o /etc/apt/sources.list
RUN curl -sSL https://ghproxy.cn/https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu noble main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN curl -o /etc/ros/rosdep/sources.list.d/20-default.list -L https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/sources.list.d/20-default.list

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN sudo apt update && sudo apt install wget python3-pip curl htop vim lsb-release gnupg -y && \
    sudo pip install vcstool2 xmacro

RUN sudo apt install ros-humble-cv-bridge

# Add Gazebo package repository and install Ignition Fortress
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && \
    apt-get install -y ignition-fortress


RUN echo 'export PATH=$PATH:/home/ws/.script' >> /home/$USERNAME/.bashrc
RUN echo 'alias wsi="source /opt/ros/humble/setup.bash"' >> /home/$USERNAME/.bashrc
RUN echo 'alias ini="source install/setup.bash"' >> /home/$USERNAME/.bashrc

USER $USERNAME

RUN export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml && rosdep update || true

# install dependencies and some tools
RUN --mount=type=bind,target=/home/ws,source=.,readonly=false cd /home/ws \
    && rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y || true


# build
# RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release

# # source entrypoint setup
# RUN sed --in-place --expression \
#     '$isource "/root/ros_ws/install/setup.bash"' \
#     /ros_entrypoint.sh

# RUN rm -rf /var/lib/apt/lists/*
