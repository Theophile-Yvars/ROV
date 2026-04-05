#!/bin/bash

# 1. Configuration ROS 2
export ROS_DOMAIN_ID=42
export PYTHONUNBUFFERED=1

# --- 2. Nettoyage ---
echo "🧹 Nettoyage des processus ROS 2..."
pkill -f "ros2" || true
pkill -f "camera_bridge" || true
pkill -f "web_video_server" || true
sleep 1

# --- 3. Chargement des sources ---
source /opt/ros/jazzy/setup.bash
if [ -f "/home/rov_ws/install/setup.bash" ]; then
    source /home/rov_ws/install/setup.bash
    echo "✅ Workspace chargé."
else
    echo "❌ Erreur : install/setup.bash non trouvé !"
    exit 1
fi

# --- 4. Vérification de la caméra virtuelle (Mode Max Quality) ---
# On vérifie que le périphérique créé par l'hôte est bien monté dans Docker
echo -n "⏳ Attente de la caméra virtuelle (/dev/video10)..."
until [ -e "/dev/video10" ]; do 
    sleep 1
    echo -n "."
done
echo " ✅ Prêt !"

# Lancement du bringup (Vérifie bien qu'il n'y a pas de doublons dans nautilus_launch.py)
ros2 launch rov_bringup nautilus_launch.py &

LAUNCH_PID=$!

echo "-------------------------------------------------------"
echo "🚀 ROV EN LIGNE (High-End V4L2 Mode)"
echo "📡 IP : $(hostname -I | awk '{print $1}') | DOMAIN : $ROS_DOMAIN_ID"
echo "-------------------------------------------------------"

trap "kill $LAUNCH_PID; exit" SIGINT SIGTERM
wait $LAUNCH_PID