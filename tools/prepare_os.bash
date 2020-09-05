#!/bin/bash

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt update && sudo apt install -y curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update && sudo apt install -y ros-foxy-ros-base ros-foxy-nav2-bringup ros-foxy-cv-bridge ros-foxy-libg2o python3-pykdl python3-rpi.gpio i2c-tools libi2c-dev git python3-argcomplete python3-colcon-common-extensions ros-foxy-test-msgs ros-foxy-py-trees-ros-viewer ros-foxy-py-trees-ros ros-foxy-py-trees-ros-interfaces ros-foxy-xacro

sudo apt remove -y --autoremove snapd unattended-upgrades plymouth

sudo systemctl disable cloud-config.service
sudo systemctl disable cloud-final.service
sudo systemctl disable cloud-init
sudo systemctl disable cloud-init-local.service

sudo systemctl disable snapd.service
sudo systemctl disable snapd.seeded.service
sudo systemctl disable snapd.socket

sudo systemctl disable systemd-networkd-wait-online.service
sudo systemctl mask systemd-networkd-wait-online.service

sudo systemctl disable snapd
sudo systemctl mask snapd

sudo sed -i 's/^ENABLED=.*/ENABLED=0/' /etc/default/motd-news

sudo systemctl stop apt-daily.timer
sudo systemctl disable apt-daily.timer
sudo systemctl mask apt-daily.service

sudo chown :i2c /dev/i2c*
sudo chmod g+rw /dev/i2c*
sudo usermod -aG i2c $USER

sudo groupadd gpio
sudo chmod g+rw /dev/gpiomem
sudo chown :gpio /dev/gpiomem
sudo chmod g+rw /dev/gpiomem
sudo usermod -aG gpio $USER

ssh-keygen -t ed25519
