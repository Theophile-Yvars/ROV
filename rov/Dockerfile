FROM ros:jazzy

# Évite les interactions bloquantes
ENV DEBIAN_FRONTEND=noninteractive

# 1. Installation des outils système et packages ROS nécessaires
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    v4l-utils \
    usbutils \
    # Packages ROS indispensables pour le Dashboard
    ros-jazzy-v4l2-camera \
    ros-jazzy-web-video-server \
    ros-jazzy-rosbridge-suite \
    && rm -rf /var/lib/apt/lists/*

# 2. Gestion des dépendances ROS
RUN rosdep update

# 3. Création du Workspace
# Note : pas besoin de faire WORKDIR deux fois, le dernier gagne
WORKDIR /home/rov_ws

# 4. Compilation initiale
# On s'assure que le dossier src existe avant de build
RUN mkdir -p src && . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

# 5. Configuration du Bash
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "if [ -f /home/rov_ws/install/setup.bash ]; then source /home/rov_ws/install/setup.bash; fi" >> ~/.bashrc

CMD ["bash"]