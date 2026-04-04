# --- Configuration ---
IMAGE_NAME = rov-jazzy
CONTAINER_NAME = rov
WS_PATH = /home/rov_ws

# Couleurs pour le terminal
YELLOW = \033[0;33m
GREEN  = \033[0;32m
NC     = \033[0m

.PHONY: help build run stop shell clean info

# Aide par défaut
help:
	@echo "$(YELLOW)Commandes ROV (Container: $(CONTAINER_NAME)) :$(NC)"
	@echo "  make build   : Construit l'image Docker via scripts/build.sh"
	@echo "  make run     : Lance le container via scripts/start_robot_docker.sh"
	@echo "  make stop    : Arrête le container $(CONTAINER_NAME)"
	@echo "  make shell   : Entre dans le container $(CONTAINER_NAME) pour coder"
	@echo "  make info    : Vérifie l'état de l'image, du container et du hardware"

# 1. Construction de l'image
build:
	@echo "$(GREEN)Construction de l'image $(IMAGE_NAME)...$(NC)"
	docker build -t $(IMAGE_NAME) .

# 2. Lancement (Appel de ton script start_robot_docker.sh)
run:
	@echo "$(GREEN)Démarrage du container $(CONTAINER_NAME)...$(NC)"
	@chmod +x scripts/start_robot_docker.sh
	@./scripts/start_robot_docker.sh

# 3. Accès interactif (C'est ici que tu passeras le plus de temps)
shell:
	@echo "$(GREEN)Connexion au shell de $(CONTAINER_NAME)...$(NC)"
	@docker exec -it $(CONTAINER_NAME) bash

# 4. Arrêt rapide
stop:
	@echo "$(YELLOW)Arrêt du container $(CONTAINER_NAME)...$(NC)"
	@docker stop $(CONTAINER_NAME) || true

# 5. Nettoyage complet
clean:
	@echo "$(YELLOW)Suppression du container et de l'image...$(NC)"
	@docker rm -f $(CONTAINER_NAME) 2>/dev/null || true
	@docker rmi $(IMAGE_NAME) 2>/dev/null || true

# 6. Commande INFO pour vérifier l'état du container, de l'image et du hardware
info:
	@echo "$(YELLOW)--- DOCKER STATUT ---$(NC)"
	@docker ps -a --filter "name=$(CONTAINER_NAME)" --format "table {{.Names}}\t{{.Status}}\t{{.Image}}"
	@echo "\n$(YELLOW)--- WORKSPACE LOCAL ---$(NC)"
	@ls src/
	@echo "\n$(YELLOW)--- HARDWARE (V4L2 & USB) ---$(NC)"
	@ls /dev/video* 2>/dev/null || echo "Aucune caméra /dev/video trouvée"
	@lsusb 2>/dev/null || echo "Commande lsusb non disponible"