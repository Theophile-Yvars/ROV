# --- Configuration ---
IMAGE_NAME     = rov-jazzy
CONTAINER_NAME = rov-container
# Chemin absolu vers la racine du workspace (2 niveaux au-dessus de src/rov)
WS_ROOT        = $(shell realpath $(CURDIR)/../..)

# Couleurs pour le terminal
YELLOW = \033[0;33m
GREEN  = \033[0;32m
RED    = \033[0;31m
NC     = \033[0m

.PHONY: help init build run stop shell clean info clean-ws reinit reset

# Aide par défaut
help:
	@echo "$(YELLOW)Commandes ROV (Container: $(CONTAINER_NAME)) :$(NC)"
	@echo "  $(GREEN)make build$(NC)    : Construit l'image Docker (L'environnement)"
	@echo "  $(GREEN)make init$(NC)     : Compile le workspace ROS 2"
	@echo "  $(GREEN)make run$(NC)      : Lance le robot (Scripts + ROS 2 Launch)"
	@echo "  $(GREEN)make shell$(NC)    : Entre dans le container pour coder/compiler"
	@echo "  $(GREEN)make stop$(NC)     : Arrête le container"
	@echo "  $(GREEN)make info$(NC)     : Vérifie l'état du système et du hardware"
	@echo "  $(RED)make clean-ws$(NC) : Supprime build/, install/ et log/"
	@echo "  $(RED)make reinit$(NC)   : Nettoyage complet du workspace + Compilation"
	@echo "  $(RED)make reset$(NC)    : La totale (Stop + Clean Image + Clean WS + Build + Init)"

# 1. Construction de l'image (OS + Dépendances)
build:
	@echo "$(GREEN)Construction de l'image $(IMAGE_NAME)...$(NC)"
	docker build -t $(IMAGE_NAME) .

# 2. Initialisation / Compilation du Workspace
init:
	@echo "🚀 Compilation via Docker (Cible: $(WS_ROOT))..."
	@docker run --rm \
		-v "$(WS_ROOT):/home/rov_ws" \
		$(IMAGE_NAME) \
		bash -c "source /opt/ros/jazzy/setup.bash && cd /home/rov_ws && colcon build --symlink-install"
	@echo "✅ Compilation terminée !"

# 3. Lancement
run:
	@echo "$(GREEN)Démarrage du système ROV...$(NC)"
	@chmod +x scripts/start_robot_docker.sh
	ROOT_DIR=$(WS_ROOT) ./scripts/start_robot_docker.sh

# 4. Accès interactif
shell:
	@echo "$(GREEN)Connexion au shell de $(CONTAINER_NAME)...$(NC)"
	@docker exec -it $(CONTAINER_NAME) bash

# 5. Arrêt
stop:
	@echo "$(YELLOW)Arrêt du container $(CONTAINER_NAME)...$(NC)"
	@docker stop $(CONTAINER_NAME) || true

# 6. Nettoyage Docker (Container et Image)
clean:
	@echo "$(RED)Suppression du container et de l'image...$(NC)"
	@docker rm -f $(CONTAINER_NAME) 2>/dev/null || true
	@docker rmi $(IMAGE_NAME) 2>/dev/null || true

# 7. Nettoyage du Workspace ROS 2
clean-ws:
	@echo "$(RED)🧹 Suppression des dossiers de build (install, build, log)...$(NC)"
	@rm -rf $(WS_ROOT)/build $(WS_ROOT)/install $(WS_ROOT)/log
	@echo "✅ Workspace nettoyé."

# 8. Réinitialisation (Clean WS + Init)
reinit: clean-ws init

# 9. Reset Total
reset: stop clean clean-ws build init

# 10. Diagnostics
info:
	@echo "$(YELLOW)--- DOCKER STATUT ---$(NC)"
	@docker ps -a --filter "name=$(CONTAINER_NAME)" --format "table {{.Names}}\t{{.Status}}\t{{.Image}}"
	@echo "\n$(YELLOW)--- WORKSPACE ROS 2 ---$(NC)"
	@ls -d $(WS_ROOT)/build $(WS_ROOT)/install $(WS_ROOT)/src 2>/dev/null || echo "Workspace non initialisé"
	@echo "\n$(YELLOW)--- CAMERAS DETECTEES (Host) ---$(NC)"
	@v4l2-ctl --list-devices 2>/dev/null || echo "v4l2-ctl non installé sur l'hôte"