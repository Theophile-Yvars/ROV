#!/bin/bash

# Fonction de nettoyage propre au Ctrl+C
cleanup() {
    echo ""
    echo "--- 🛑 Arrêt du Robot (Caméra + Container) ---"
    kill $CAM_PID 2>/dev/null
    docker stop robotchenilles 2>/dev/null
    exit
}

trap cleanup SIGINT

# --- 0. Synchronisation du Temps ---
echo "--- 🕒 Synchronisation de l'horloge ---"
sudo systemctl restart systemd-timesyncd 2>/dev/null
sleep 2

# --- 1. Caméra (Flux UDP pour le Web Video Server) ---
echo "--- 📷 Lancement de la caméra (Hôte) ---"
pkill -9 rpicam-vid 2>/dev/null
rpicam-vid -t 0 --width 640 --height 480 --framerate 30 --codec mjpeg -o udp://0.0.0.0:5000 --inline --nopreview &
CAM_PID=$!

# --- 2. Docker ---
echo "--- 🐳 Démarrage du Container ---"
docker rm -f robotchenilles 2>/dev/null
docker run -dt --name robotchenilles \
  --privileged --net=host --ipc=host --pid=host \
  --shm-size=1gb \
  -v /dev:/dev -v /sys:/sys -v /run:/run \
  -v /usr/lib/aarch64-linux-gnu:/host_libs:ro \
  -v /usr/bin:/host_bins:ro \
  -v /usr/share/libcamera:/usr/share/libcamera:ro \
  -v ~/robot_ws:/home/robot_ws \
  -v /etc/timezone:/etc/timezone:ro \
  -v /etc/localtime:/etc/localtime:ro \
  robot-jazzy-pi5

# --- 3. Hardware link ---
docker exec robotchenilles sh -c "echo '/host_libs' > /etc/ld.so.conf.d/host.conf && ldconfig"

# --- 4. Launch ROS 2 (Via le nouveau Launcher Python) ---
echo "--- 🚀 Lancement de ROS 2 (Unified Launcher) ---"
# On compile (build) puis on lance le fichier python unique
docker exec -it robotchenilles bash -c "
    source /opt/ros/jazzy/setup.bash && \
    cd /home/robot_ws && \
    colcon build --symlink-install && \
    source install/setup.bash && \
    ros2 launch robot_bringup robot.launch.py
"

cleanup