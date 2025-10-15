#!/bin/bash

# Actualitza l'índex de paquets
sudo apt update

# Instal·la paquets necessaris
sudo apt install -y ca-certificates curl gnupg lsb-release

# Crea el directori per a la clau GPG de Docker
sudo install -m 0755 -d /etc/apt/keyrings

# Afegeix la clau GPG oficial de Docker
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# Afegeix el repositori oficial de Docker
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Torna a actualitzar l'índex de paquets
sudo apt update

# Instal·la Docker Engine i plugins
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Instal·la docker-compose com a binari standalone (opcional)
sudo curl -L "https://github.com/docker/compose/releases/latest/download/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose

# Activa i inicia els serveis de Docker
sudo systemctl enable docker
sudo systemctl enable containerd
sudo systemctl start docker

# Afegeix l'usuari actual al grup docker per evitar usar sudo
sudo usermod -aG docker $USER

# Recomanat: reiniciar per aplicar els canvis de grup
sudo reboot
