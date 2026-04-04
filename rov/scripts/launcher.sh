#!/bin/bash

# =================================================================
# 🚀 LAUNCHER ROBOT CHENILLES v2.1 - FIX 90° & AUTO-CHECK
# =================================================================

# 1. Configuration ROS 2
export ROS_DOMAIN_ID=42
export PYTHONUNBUFFERED=1
# Force le frame ID pour le driver LD19 (selon les versions du driver)
export LD19_FRAME="base_laser" 

# --- FONCTIONS DE VÉRIFICATION ---
wait_for_device() {
    local device=$1
    echo -n "⏳ Attente du matériel ($device)..."
    while [ ! -e "$device" ]; do sleep 1; echo -n "."; done
    echo " ✅ Détecté !"
}

wait_for_topic() {
    local topic=$1
    echo -n "⏳ Attente du flux ROS 2 ($topic)..."
    until ros2 topic list 2>/dev/null | grep -q "$topic"; do sleep 1; echo -n "."; done
    echo " ✅ Flux actif !"
}

# 2. Nettoyage
echo "🧹 Nettoyage des processus ROS 2..."
pkill -f "ros2" || true
pkill -f "rosbridge" || true
sleep 2

# 3. Chargement des sources
source /opt/ros/jazzy/setup.bash
cd /home/robot_ws
source install/setup.bash

echo "--- 1. Initialisation LiDAR LD19 ---"
wait_for_device "/dev/ttyUSB0"
sudo chmod 777 /dev/ttyUSB0

# Lancement du Driver avec forçage du frame_id
ros2 launch ldlidar_stl_ros2 ld19.launch.py frame_id:=base_laser &

# On attend que le LiDAR publie réellement
wait_for_topic "/scan"

echo "--- 2. Correction Géométrique (TF) ---"
# odom -> base_link
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link &
sleep 1

# base_link -> base_laser (Correction de 90° : +1.5708 rad)
# Format : x y z yaw pitch roll
echo "📡 Application de la correction +90° sur base_laser..."
ros2 run tf2_ros static_transform_publisher 0 0 0.18 1.5708 0 0 base_link base_laser &

echo "--- 3. Lancement Hardware (Moteurs, Caméra, Tilt) ---"
ros2 launch robot_bringup robot.launch.py &

echo "-------------------------------------------------------"
echo "✅ SYSTÈME OPÉRATIONNEL"
echo "🛠  Correction Angle : +90° (1.5708 rad)"
echo "📡 IP : $(hostname -I | awk '{print $1}')"
echo "-------------------------------------------------------"

wait