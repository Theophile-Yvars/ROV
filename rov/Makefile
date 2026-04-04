# --- Configuration ---
IMAGE_NAME = rov-jazzy
CONTAINER_NAME = rov
WS_PATH = /home/rov_ws

# Couleurs pour le terminal
YELLOW = \033[0;33m
GREEN  = \033[0;32m
NC     = \033[0m

.PHONY: help init build run stop shell clean info

# Aide par défaut
help:
	@echo "$(YELLOW)Commandes ROV (Container: $(CONTAINER_NAME)) :$(NC)"
	@echo "  make build   : 1. Construit l'image Docker (L'environnement)"
	@echo "  make init    : 2. Initialise le Workspace (Compilation initiale)"
	@echo "  make run     : 3. Lance le robot (Scripts + ROS 2 Launch)"
	@echo "  make shell   : Entre dans le container pour coder/compiler"
	@echo "  make stop    : Arrête le container $(CONTAINER_NAME)"
	@echo "  make info    : Vérifie l'état du système et du hardware"
	@echo "  make clean   : Supprime le container et l'image"

# 1. Construction de l'image (OS + Dépendances)
build:
	@echo "$(GREEN)Construction de l'image $(IMAGE_NAME)...$(NC)"
	docker build -t $(IMAGE_NAME) .

# 2. Initialisation du Workspace sur la Pi
init:
	@echo "🚀 Compilation via Docker (car ROS 2 n'est pas sur la Pi)..."
	@docker run --rm \
		-v "$(shell realpath $(CURDIR)/../..):/home/rov_ws" \
		$(IMAGE_NAME) \
		bash -c "source /opt/ros/jazzy/setup.bash && cd /home/rov_ws && colcon build --symlink-install"
	@echo "✅ Compilation terminée ! Les dossiers 'install' et 'build' sont sur ta Pi."

# 3. Lancement (Appel de ton script start_robot_docker.sh)
run:
	@echo "$(GREEN)Démarrage du container $(CONTAINER_NAME)...$(NC)"
	@chmod +x scripts/start_robot_docker.sh
	@./scripts/start_robot_docker.sh

# 4. Accès interactif
shell:
	@echo "$(GREEN)Connexion au shell de $(CONTAINER_NAME)...$(NC)"
	@docker exec -it $(CONTAINER_NAME) bash

# 5. Arrêt rapide
stop:
	@echo "$(YELLOW)Arrêt du container $(CONTAINER_NAME)...$(NC)"
	@docker stop $(CONTAINER_NAME) || true

# 6. Nettoyage complet
clean:
	@echo "$(YELLOW)Suppression du container et de l'image...$(NC)"
	@docker rm -f $(CONTAINER_NAME) 2>/dev/null || true
	@docker rmi $(IMAGE_NAME) 2>/dev/null || true

# 7. Commande INFO
info:
	@echo "$(YELLOW)--- DOCKER STATUT ---$(NC)"
	@docker ps -a --filter "name=$(CONTAINER_NAME)" --format "table {{.Names}}\t{{.Status}}\t{{.Image}}"
	@echo "\n$(YELLOW)--- WORKSPACE LOCAL ---$(NC)"
	@ls -d build install log src 2>/dev/null || echo "Workspace non initialisé (faites 'make init')"
	@echo "\n$(YELLOW)--- HARDWARE (V4L2 & USB) ---$(NC)"
	@ls /dev/video* 2>/dev/null || echo "Aucune caméra /dev/video trouvée"
	@lsusb 2>/dev/null || echo "Commande lsusb non disponible"