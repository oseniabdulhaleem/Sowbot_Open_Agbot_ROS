#!/bin/bash
# Exit on error
set -e

echo "🚮 Removing all legacy build artifacts and logs..."
sudo rm -rf build/ install/ log/

echo "🐳 Pruning Docker to remove stale layers and containers..."
# This stops containers and removes images associated with this project
docker compose down --rmi local --volumes --remove-orphans

echo "🔌 Setting ThinkPad USB permissions..."
sudo chmod 666 /dev/ttyACM* || echo "⚠️ No /dev/ttyACM devices found. Check your ESP32/GPS connections."

echo "🏗️ Rebuilding containers from scratch (this may take a while)..."
# --no-cache ensures it doesn't reuse any broken Jetson-era layers
docker compose build --no-cache

echo "🚀 Starting Open Agbot..."
docker compose up
