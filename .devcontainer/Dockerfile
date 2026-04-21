FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV CMAKE_PREFIX_PATH=/usr/local:${CMAKE_PREFIX_PATH}
ENV LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH}
ENV ROS_DISTRO=humble
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

SHELL ["/bin/bash", "-c"]

# Base tools
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    pkg-config \
    git \
    wget \
    curl \
    nano \
    htop \
    sudo \
    tmux \
    vim \
    less \
    unzip \
    zip \
    rsync \
    iputils-ping \
    net-tools \
    usbutils \
    mesa-utils \
    software-properties-common \
    python3-pip \
    python3-tk \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# rosdep / colcon
RUN rosdep init || true && \
    rosdep update --rosdistro ${ROS_DISTRO} && \
    colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml || true && \
    colcon mixin update || true

# ROS 2 + simulation + perception + control
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-xacro \
    ros-humble-rviz2 \
    ros-humble-plotjuggler-ros \
    ros-humble-rqt \
    ros-humble-rqt-controller-manager \
    ros-humble-rqt-graph \
    ros-humble-rqt-tf-tree \
    ros-humble-joint-state-publisher-gui \
    ros-humble-message-filters \
    ros-humble-tf2-ros \
    ros-humble-tf-transformations \
    ros-humble-robot-localization \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-vision-opencv \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-controller-manager \
    ros-humble-joint-state-broadcaster \
    ros-humble-hardware-interface \
    ros-humble-transmission-interface \
    ros-humble-mavros \
    ros-humble-mavros-extras \
    ros-humble-mavros-msgs \
    && rm -rf /var/lib/apt/lists/*

# Native libs
RUN apt-get update && apt-get install -y --no-install-recommends \
    libglew-dev \
    libglm-dev \
    libsdl2-dev \
    libbullet-dev \
    libopencv-dev \
    python3-opencv \
    libeigen3-dev \
    libpcl-dev \
    libyaml-cpp-dev \
    libapr1-dev \
    libaprutil1-dev \
    default-jre \
    default-jdk \
    python3-wxgtk4.0 \
    && rm -rf /var/lib/apt/lists/*

# GeographicLib datasets for MAVROS
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    chmod +x install_geographiclib_datasets.sh && \
    ./install_geographiclib_datasets.sh && \
    rm install_geographiclib_datasets.sh

# Python packages
RUN pip3 install --no-cache-dir \
    pymavlink \
    MAVProxy \
    future \
    transforms3d

# GCC/G++ 13
RUN add-apt-repository ppa:ubuntu-toolchain-r/test -y && \
    apt-get update && \
    apt-get install -y gcc-13 g++-13 && \
    update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-13 100 && \
    update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-13 100 && \
    rm -rf /var/lib/apt/lists/*

# Sophus
RUN git clone https://github.com/strasdat/Sophus.git /tmp/Sophus && \
    cd /tmp/Sophus && \
    git checkout 1.22.10 && \
    cmake -S . -B build \
      -DSOPHUS_USE_BASIC_LOGGING=ON \
      -DBUILD_SOPHUS_TESTS=OFF \
      -DBUILD_SOPHUS_EXAMPLES=OFF \
      -DCMAKE_BUILD_TYPE=Release && \
    cmake --build build -j4 && \
    cmake --install build && \
    ldconfig && \
    rm -rf /tmp/Sophus

# Livox-SDK2
RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2 && \
    cd /tmp/Livox-SDK2 && \
    sed -i '1i #include <cstdint>' sdk_core/comm/define.h && \
    sed -i '31i #include <cstdint>' sdk_core/logger_handler/file_manager.h && \
    mkdir build && cd build && \
    cmake .. && \
    make -j4 && \
    make install && \
    ldconfig && \
    rm -rf /tmp/Livox-SDK2

# Micro-XRCE-DDS-Gen
RUN git clone --recurse-submodules https://github.com/eProsima/Micro-XRCE-DDS-Gen.git /opt/Micro-XRCE-DDS-Gen && \
    cd /opt/Micro-XRCE-DDS-Gen && \
    ./gradlew assemble && \
    ln -s /opt/Micro-XRCE-DDS-Gen/scripts/microxrceddsgen /usr/local/bin/microxrceddsgen

# User
ARG USERNAME=mario-cirtesu
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid ${USER_GID} ${USERNAME} && \
    groupadd --gid 107 input-host || true && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} && \
    usermod -aG 107 ${USERNAME} && \
    echo "${USERNAME} ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} && \
    chmod 0440 /etc/sudoers.d/${USERNAME}

RUN echo "source /opt/ros/humble/setup.bash" >> /home/${USERNAME}/.bashrc && \
    echo "export CMAKE_PREFIX_PATH=/usr/local:\$CMAKE_PREFIX_PATH" >> /home/${USERNAME}/.bashrc && \
    echo "export LD_LIBRARY_PATH=/usr/local/lib:\$LD_LIBRARY_PATH" >> /home/${USERNAME}/.bashrc && \
    echo "export ROS_DOMAIN_ID=0" >> /home/${USERNAME}/.bashrc

USER ${USERNAME}
WORKDIR /home/${USERNAME}/blueboat_ws