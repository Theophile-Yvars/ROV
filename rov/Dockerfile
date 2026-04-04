FROM ros:jazzy

# Évite les interactions bloquantes
ENV DEBIAN_FRONTEND=noninteractive

# 1. Installation des outils de base
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-colcon-common-extensions \
    v4l-utils \
    usbutils \
    && rm -rf /var/lib/apt/lists/*

# 2. Gestion des dépendances ROS (rosdep)
# Indispensable pour que ROS trouve automatiquement les libs de tes nodes
RUN rosdep update

# 3. Création du Workspace
WORKDIR /home/rov_ws/src
WORKDIR /home/rov_ws

# 4. Compilation initiale
# On crée le dossier install pour éviter les erreurs de "source" au premier lancement
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

# 5. Configuration du Bash
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "if [ -f /home/rov_ws/install/setup.bash ]; then source /home/rov_ws/install/setup.bash; fi" >> ~/.bashrc

CMD ["bash"]