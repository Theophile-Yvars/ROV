FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive

# 1. Installation des dépendances (Correction du placement de rm -rf)
RUN apt-get update && apt-get install -y \
    liblgpio-dev \
    libcamera-ipa \
    libcamera0.2 \
    libcamera-tools \
    python3-pip \
    python3-gpiozero \
    python3-lgpio \
    ros-jazzy-rosbridge-server \
    ros-jazzy-web-video-server \
    ros-jazzy-camera-ros \
    ros-jazzy-cv-bridge \
    v4l-utils \
    ros-jazzy-slam-toolbox \
    ros-jazzy-nav2-map-server \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-geometry-msgs \
    git \
    python3-colcon-common-extensions \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /usr/lib/aarch64-linux-gnu/libcamera
ENV LIBCAMERA_IPA_MODULE_PATH=/usr/lib/aarch64-linux-gnu/libcamera

WORKDIR /home/robot_ws

# 2. Récupération du driver
RUN mkdir -p src && cd src && \
    git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git

# 3. PATCH CRITIQUE : Correction du chemin pour le sed
# On utilise le chemin relatif par rapport au WORKDIR
RUN sed -i '1i #include <pthread.h>' src/ldlidar_stl_ros2/ldlidar_driver/src/logger/log_module.cpp

# 4. Compilation
# On force le sourçage de ROS Jazzy pour que colcon connaisse les commandes
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

# 5. Sourçage automatique pour tes futurs terminaux SSH
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /home/robot_ws/install/setup.bash" >> /root/.bashrc