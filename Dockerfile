FROM ros:humble-ros-base

ARG USERNAME=cvdoc
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

RUN sudo apt update && \
    sudo apt install python3-pip curl wget htop vim lsb-release gnupg -y && \
    sudo pip install vcstool2 xmacro

# setup zsh
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.2.1/zsh-in-docker.sh)" -- \
    -t jispwoso -p git \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-syntax-highlighting && \
    chsh -s /bin/zsh
CMD [ "/bin/zsh" ]

# Add Gazebo package repository and install Ignition Fortress
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && \
    apt-get install -y ignition-fortress

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# setup .zshrc
RUN echo 'export TERM=xterm-256color\n\
    eval "$(register-python-argcomplete3 ros2)"\n\
    eval "$(register-python-argcomplete3 colcon)"\n'\
    >> /home/$USERNAME/.zshrc
RUN echo 'export PATH=$PATH:/home/ws/.script' >> /home/$USERNAME/.zshrc
RUN echo 'alias wsi="source /opt/ros/humble/setup.zsh"' >> /home/$USERNAME/.zshrc
RUN echo 'alias ini="source install/setup.zsh"' >> /home/$USERNAME/.zshrc

USER $USERNAME

# install dependencies and some tools
RUN --mount=type=bind,target=/home/ws,source=.,readonly=false cd /home/ws \
    && rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# build
# RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=release

# # source entrypoint setup
# RUN sed --in-place --expression \
#     '$isource "/root/ros_ws/install/setup.bash"' \
#     /ros_entrypoint.sh

# RUN rm -rf /var/lib/apt/lists/*
