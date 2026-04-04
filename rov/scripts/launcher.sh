#!/bin/bash

# =================================================================
# 🚀 LAUNCHER ROV v3.0 - ROS 2 JAZZY (WS: rov_ws)
# =================================================================

# 1. Configuration ROS 2
# Le Domain ID permet d'isoler ton ROV si d'autres robots sont sur le même réseau
export ROS_DOMAIN_ID=42
export PYTHONUNBUFFERED=1

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

# 2. Nettoyage des résidus ROS 2
echo "🧹 Nettoyage des processus ROS 2 précédents..."
pkill -f "ros2" || true
pkill -f "rosbridge" || true
sleep 1

# 3. Chargement des sources (Chemins mis à jour vers rov_ws)
source /opt/ros/jazzy/setup.bash
if [ -f "/home/rov_ws/install/setup.bash" ]; then
    source /home/rov_ws/install/setup.bash
else
    echo "⚠️ Attention : Workspace non compilé. Lancement impossible."
    exit 1
fi

echo "--- 3. Lancement Hardware & Drivers ---"

# Exemple d'utilisation de tes fonctions de check :
# wait_for_device "/dev/video0" # Si tu as une caméra USB
# wait_for_device "/dev/ttyACM0" # Si tu as une carte Arduino/Pixhawk

# Lancement du bringup (nom de package mis à jour : rov_bringup)
ros2 launch rov_bringup robot.launch.py &

# On récupère le PID du launch pour pouvoir le tuer proprement
LAUNCH_PID=$!

echo "-------------------------------------------------------"
echo "✅ SYSTÈME ROV OPÉRATIONNEL"
echo "📡 IP INTERNE : $(hostname -I | awk '{print $1}')"
echo "📡 DOMAIN ID  : $ROS_DOMAIN_ID"
echo "-------------------------------------------------------"

# Gestion propre de l'arrêt
trap "kill $LAUNCH_PID; exit" SIGINT SIGTERM

wait $LAUNCH_PID