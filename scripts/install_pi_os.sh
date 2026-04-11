#!/usr/bin/env bash
# Arespath Rover — Raspberry Pi dependency installer
set -e

echo "Updating packages..."
sudo apt-get update -qq

echo "Installing system packages..."
sudo apt-get install -y python3-pip python3-dev python3-numpy \
  libatlas-base-dev libopenblas-dev \
  libavcodec-dev libavformat-dev libswscale-dev \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev

echo "Installing Python packages..."
pip3 install --break-system-packages \
  Flask Flask-SocketIO simple-websocket pyserial numpy Pillow PyYAML

echo "Installing OpenCV (headless)..."
pip3 install --break-system-packages opencv-python-headless

echo "Done. Flash the Arduino sketch, edit app/config.py, then run:"
echo "  python3 -m app.main"
