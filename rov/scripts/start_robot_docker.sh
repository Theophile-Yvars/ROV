#!/bin/bash

# --- Configuration ---
CONTAINER_NAME="rov-container"
IMAGE_NAME="rov-jazzy"
WS_PATH="/home/rov_ws"

cleanup() {
    echo ""
    echo "--- 🛑 Arrêt du Robot ---"
    # On tue proprement les processus de flux
    sudo pkill -9 rpicam-vid 2>/dev/null
    sudo pkill -9 ffmpeg 2>/dev/null
    docker stop $CONTAINER_NAME 2>/dev/null
    exit
}

# Capture le CTRL+C pour tout arrêter proprement
trap cleanup SIGINT SIGTERM

# --- 0. Activation du Driver ---
echo "--- 🛠️  Réinitialisation du driver v4l2loopback ---"
sudo pkill -9 rpicam-vid 2>/dev/null
sudo pkill -9 ffmpeg 2>/dev/null
sudo modprobe -r v4l2loopback 2>/dev/null
sleep 1

# Chargement avec exclusive_caps=1 (indispensable pour Chrome/OpenCV/ROS)
sudo modprobe v4l2loopback devices=1 video_nr=10 card_label="ROV_CAM" exclusive_caps=1
sudo chmod 666 /dev/video10

# --- 1. Lancement du flux Vidéo (Pipeline Optimisé Pi 5) ---
echo "--- 📷 Capture IMX708 -> /dev/video10 (Format YUYV) ---"

# Pipeline sans '-re' pour éviter la latence et avec 'yuyv422' en sortie
# 'v4l2-ctl' n'est plus nécessaire avant car FFmpeg va négocier le format direct.
rpicam-vid -t 0 --width 1280 --height 720 --framerate 30 --nopreview --codec yuv420 -o - | \
ffmpeg -v error -f rawvideo -pixel_format yuv420p -video_size 1280x720 -i - \
       -f v4l2 -pix_fmt yuyv422 /dev/video10 > /dev/null 2>&1 &

# Pause de sécurité pour laisser FFmpeg "ouvrir" le device
sleep 3
echo "✅ Pipeline stabilisé."

# --- 2. Lancement Docker ---
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

# --- 3. Build et Lancement interne ---
echo "--- 🏗️  Build du workspace ---"
docker exec $CONTAINER_NAME chmod +x $WS_PATH/src/rov/scripts/build.sh
docker exec -it $CONTAINER_NAME bash -c "$WS_PATH/src/rov/scripts/build.sh"

# Fix pour les libs hôte si nécessaire
docker exec $CONTAINER_NAME sh -c "echo '/host_libs' > /etc/ld.so.conf.d/host.conf && ldconfig"

echo "--- 🚀 Lancement de la stack ROV ---"
docker exec $CONTAINER_NAME chmod +x $WS_PATH/src/rov/src/rov-hardware/src/camera_bridge.py
docker exec $CONTAINER_NAME chmod +x $WS_PATH/src/rov/scripts/launcher.sh
docker exec -it $CONTAINER_NAME bash -c "$WS_PATH/src/rov/scripts/launcher.sh"

# Garde le script en vie pour le trap cleanup
cleanup