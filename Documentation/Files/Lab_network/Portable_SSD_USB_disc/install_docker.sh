#!/bin/bash

# Update package index
echo "Updating package index..."
sudo apt update

# Install required packages
echo "Installing required packages..."
sudo apt install -y ca-certificates curl gnupg

# Add Docker’s official GPG key
echo "Adding Docker's GPG key..."
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg |   sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# Set up Docker repository
echo "Setting up Docker repository..."
echo   "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg]   https://download.docker.com/linux/ubuntu   $(lsb_release -cs) stable" |   sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Update package index again
echo "Updating package index after adding Docker repo..."
sudo apt update

# Install Docker Engine and plugins
echo "Installing Docker Engine and plugins..."
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Verify Docker installation
echo "Verifying Docker installation..."
docker --version

# Install docker-compose standalone binary
echo "Installing docker-compose standalone binary..."
sudo curl -L "https://github.com/docker/compose/releases/latest/download/docker-compose-$(uname -s)-$(uname -m)"   -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose

# Verify docker-compose installation
echo "Verifying docker-compose installation..."
docker-compose --version

echo "Docker and docker-compose installation completed successfully."
