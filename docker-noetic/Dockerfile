# ===============================
# ROS Noetic Dev Image (Desktop)
# ===============================
FROM osrf/ros:noetic-desktop-full

ENV DEBIAN_FRONTEND=noninteractive \
    TERM=xterm-256color \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=all \
    QT_X11_NO_MITSHM=1 \
    LIBGL_ALWAYS_INDIRECT=0

WORKDIR /root

# 작업공간 복사
COPY . /root/adi_ws

# Common dev tools + ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    vim \
    tmux \
    nano \
    sudo \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    libeigen3-dev \
    libopencv-dev \
    python3-opencv \
    libpcl-dev \
    ros-noetic-pcl-ros \
    ros-noetic-pcl-conversions \
    ros-noetic-openslam-gmapping \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-nav-msgs \
    ros-noetic-vision-opencv \
    ros-noetic-cv-bridge \
    ros-noetic-image-transport \
    ros-noetic-geographic-msgs \
    ros-noetic-diagnostic-updater \
    ros-noetic-image-geometry \
    ros-noetic-camera-info-manager \
    ros-noetic-serial \
    ros-noetic-vision-msgs \
    libgeographic-dev \
    geographiclib-tools \
    ros-noetic-hector-slam \
    ros-noetic-sparse-bundle-adjustment \
    ros-noetic-map-server \
    ros-noetic-ackermann-msgs \
    ros-noetic-navigation \
    ros-noetic-teb-local-planner \
    ros-noetic-rviz \
    ros-noetic-rqt \
    ros-noetic-rqt-graph \
    ros-noetic-rqt-plot \
    v4l-utils \
    ffmpeg \
    gstreamer1.0-tools \
    gstreamer1.0-libav \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    udev \
    usbutils \
    less \
    htop \
    tzdata \
    locales \
    gnupg2 \
    lsb-release \
    ca-certificates \
    iproute2 \
    net-tools \
    iputils-ping \
    netcat \
    openssh-client \
    openssh-server \
    mesa-utils \
    libgl1-mesa-glx \
    libglu1-mesa \
 && rm -rf /var/lib/apt/lists/*

# Python utility packages
RUN python3 -m pip install --no-cache-dir \
    numpy \
    matplotlib \
    tqdm \
    scikit-learn

# Python 3.10 (소스 빌드 - deadsnakes PPA 미지원 환경 대응)
RUN apt-get update && apt-get install -y --no-install-recommends \
    zlib1g-dev \
    libncurses5-dev \
    libgdbm-dev \
    libnss3-dev \
    libssl-dev \
    libreadline-dev \
    libffi-dev \
    libsqlite3-dev \
    libbz2-dev \
 && rm -rf /var/lib/apt/lists/* \
 && wget https://www.python.org/ftp/python/3.10.14/Python-3.10.14.tgz \
 && tar -xf Python-3.10.14.tgz \
 && cd Python-3.10.14 \
 && ./configure --enable-optimizations \
 && make -j$(nproc) \
 && make altinstall \
 && cd .. && rm -rf Python-3.10.14 Python-3.10.14.tgz

# pip for python3.10 + moondream (torch 포함 자동 설치)
RUN curl -sS https://bootstrap.pypa.io/get-pip.py | python3.10 \
 && python3.10 -m pip install --no-cache-dir \
    moondream \
    opencv-python \
    rospkg \
    catkin-pkg

# rosdep 초기화가 안 되어 있을 때만 실행
RUN rosdep update || true

# ROS setup
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# 작업공간 기본 위치
WORKDIR /root/adi_ws

CMD ["bash"]
