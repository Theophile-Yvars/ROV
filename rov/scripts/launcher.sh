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

# --- 3. Chargement des sources ---
# On source ROS 2 global (installé dans l'image Docker)
source /opt/ros/jazzy/setup.bash

# On détecte où se trouve le dossier install
# Si on est dans le container, c'est /home/rov_ws
# Si on est sur la Pi, c'est là où tu as lancé le script
if [ -f "/home/rov_ws/install/setup.bash" ]; then
    source /home/rov_ws/install/setup.bash
    echo "✅ Workspace /home/rov_ws chargé."
elif [ -f "$HOME/rov-ws/install/setup.bash" ]; then
    source "$HOME/rov-ws/install/setup.bash"
    echo "✅ Workspace ~/rov-ws chargé."
else
    echo "❌ Erreur : Impossible de trouver install/setup.bash"
    echo "As-tu bien fait un 'make init' réussi ?"
    exit 1
fi

# Exemple d'utilisation de tes fonctions de check :
# wait_for_device "/dev/video0" # Si tu as une caméra USB
# wait_for_device "/dev/ttyACM0" # Si tu as une carte Arduino/Pixhawk

# Lancement du bringup (nom de package mis à jour : rov_bringup)
ros2 launch rov_bringup robot.launch.py &

# On récupère le PID du launch pour pouvoir le tuer proprement
LAUNCH_PID=$!

echo "-------------------------------------------------------"
echo "🚀 ROV EN LIGNE (Package: rov_bringup)"
echo "📡 IP : $(hostname -I | awk '{print $1}') | DOMAIN : $ROS_DOMAIN_ID"
echo "-------------------------------------------------------"

# Gestion propre de l'arrêt
trap "kill $LAUNCH_PID; exit" SIGINT SIGTERM

wait $LAUNCH_PID