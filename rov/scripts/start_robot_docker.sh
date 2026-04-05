#!/bin/bash

# --- Configuration ---
CONTAINER_NAME="rov-container"
IMAGE_NAME="rov-jazzy"
WS_PATH="/home/rov_ws"

cleanup() {
    echo ""
    echo "--- 🛑 Arrêt du Robot ---"
    pkill -9 rpicam-vid 2>/dev/null
    pkill -9 ffmpeg 2>/dev/null
    docker stop $CONTAINER_NAME 2>/dev/null
    exit
}

trap cleanup SIGINT SIGTERM

# --- 0. Synchronisation du Temps ---
echo "--- 🕒 Synchronisation de l'horloge hôte ---"
sudo systemctl restart systemd-timesyncd 2>/dev/null
sleep 1

# --- 1. Lancement du flux Haute Qualité ---
echo "--- 📷 Initialisation IMX708 Haute Qualité (1080p) ---"
pkill -9 rpicam-vid 2>/dev/null
pkill -9 ffmpeg 2>/dev/null
sleep 1

sudo chmod 666 /dev/video10 2>/dev/null

# On utilise rpicam-vid pour sortir du H264 brut, et on force ffmpeg à sortir en v4l2
rpicam-vid -t 0 --width 1920 --height 1080 --framerate 30 --inline --nopreview --codec h264 -o - | \
ffmpeg -f h264 -i - -vcodec copy -f v4l2 /dev/video10 > /dev/null 2>&1 &

sleep 2
echo "✅ Caméra virtuelle 1080p prête sur /dev/video10"

# --- 2. Nettoyage et Lancement Docker ---
echo "--- 🐳 Démarrage du Container : $CONTAINER_NAME ---"
docker rm -f $CONTAINER_NAME 2>/dev/null
REAL_WS_ROOT=$(realpath "$(pwd)/../..")

docker run -dt --name $CONTAINER_NAME \
  --privileged \
  --net=host --ipc=host --pid=host \
  --group-add video \
  -v /dev:/dev \
  -v /sys:/sys \
  -v "$REAL_WS_ROOT:/home/rov_ws" \
  $IMAGE_NAME

# --- 3. Build et Liaison ---
echo "--- 🛠️  Phase de Build Interne ---"
docker exec $CONTAINER_NAME chmod +x $WS_PATH/src/rov/scripts/build.sh
docker exec -it $CONTAINER_NAME bash -c "$WS_PATH/src/rov/scripts/build.sh"

echo "--- 🔗 Liaison des bibliothèques hôtes ---"
docker exec $CONTAINER_NAME sh -c "echo '/host_libs' > /etc/ld.so.conf.d/host.conf && ldconfig"

# --- 4. Lancement du Launcher ROS 2 ---
echo "--- 🚀 Appel du launcher interne ---"
# On s'assure que le bridge a les droits d'exécution
docker exec $CONTAINER_NAME chmod +x $WS_PATH/src/rov/src/rov-hardware/src/camera_bridge.py
docker exec $CONTAINER_NAME chmod +x $WS_PATH/src/rov/scripts/launcher.sh
docker exec -it $CONTAINER_NAME bash -c "$WS_PATH/src/rov/scripts/launcher.sh"

cleanup