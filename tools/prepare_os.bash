#!/bin/bash

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt update && sudo apt install -y curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update && sudo apt install -y ros-eloquent-desktop i3 xinit i2c-tools libi2c-dev git lxterminal python3-argcomplete python3-colcon-common-extensions ros-eloquent-launch-testing ros-eloquent-behaviortree-cpp-v3 libgraphicsmagick++1-dev lcov libsdl1.2-dev libsdl-image1.2-dev ros-eloquent-test-msgs ros-eloquent-gazebo-ros-pkgs v4l-utils

sudo curl -L --output /usr/bin/rpi-update https://raw.githubusercontent.com/Hexxeh/rpi-update/master/rpi-update && sudo chmod +x /usr/bin/rpi-update

sudo apt remove -y snapd unattended-upgrades

sudo systemctl disable cloud-config.service
sudo systemctl disable cloud-final.service
sudo systemctl disable cloud-init
sudo systemctl disable cloud-init-local.service

sudo systemctl disable snapd.service
sudo systemctl disable snapd.seeded.service
sudo systemctl disable snapd.socket

sudo systemctl disable systemd-networkd-wait-online.service
sudo systemctl mask systemd-networkd-wait-online.service

sudo systemctl disable lxd-containers.service
sudo systemctl mask lxd-containers.service

sudo systemctl disable lxd
sudo systemctl mask lxd
sudo systemctl disable snapd
sudo systemctl mask snapd
sudo systemctl disable NetworkManager-wait-online.service
sudo systemctl mask NetworkManager-wait-online.service


sudo chown :i2c /dev/i2c*
sudo chmod g+rw /dev/i2c*
sudo usermod -aG i2c $USER

ssh-keygen -t ed25519
