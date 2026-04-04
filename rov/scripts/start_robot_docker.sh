#!/bin/bash

# --- Configuration des noms ---
CONTAINER_NAME="rov-container"
IMAGE_NAME="rov-jazzy"
WS_PATH="/home/rov_ws"

# Fonction de nettoyage propre au Ctrl+C
cleanup() {
    echo ""
    echo "--- 🛑 Arrêt du Robot (Nettoyage du container $CONTAINER_NAME) ---"
    docker stop $CONTAINER_NAME 2>/dev/null
    exit
}

trap cleanup SIGINT

# --- 0. Synchronisation du Temps ---
echo "--- 🕒 Synchronisation de l'horloge hôte ---"
sudo systemctl restart systemd-timesyncd 2>/dev/null
sleep 1

# --- 1. Nettoyage et Lancement Docker ---
echo "--- 🐳 Démarrage du Container : $CONTAINER_NAME ---"
docker rm -f $CONTAINER_NAME 2>/dev/null

REAL_WS_ROOT=$(realpath "$(pwd)/../..")

docker run -dt --name $CONTAINER_NAME \
  --privileged --net=host --ipc=host --pid=host \
  --shm-size=1gb \
  -v /dev:/dev -v /sys:/sys -v /run:/run \
  -v /usr/lib/aarch64-linux-gnu:/host_libs:ro \
  -v /usr/bin:/host_bins:ro \
  -v /usr/share/libcamera:/usr/share/libcamera:ro \
  -v "/home/yvars/rov-ws:/home/rov_ws" \
  -v /etc/timezone:/etc/timezone:ro \
  -v /etc/localtime:/etc/localtime:ro \
  $IMAGE_NAME

# --- 2. Hardware Link ---
echo "--- 🔗 Liaison des bibliothèques hôtes ---"
docker exec $CONTAINER_NAME sh -c "echo '/host_libs' > /etc/ld.so.conf.d/host.conf && ldconfig"

# --- 3. Lancement du Launcher ROS 2 ---
echo "--- 🚀 Appel du launcher interne ---"
# 1. On s'assure que le script est exécutable à l'intérieur
docker exec $CONTAINER_NAME chmod +x $WS_PATH/src/rov/scripts/launcher.sh
# 2. On lance le script (Note le chemin sans /src/)
docker exec -it $CONTAINER_NAME bash -c "$WS_PATH/src/rov/scripts/launcher.sh"

cleanup