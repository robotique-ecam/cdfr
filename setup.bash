#!/bin/bash

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt update && sudo apt install -y curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update && sudo apt install -y ros-eloquent-desktop i3 xinit i2c-tools libi2c-dev git lxterminal python3-argcomplete python3-colcon-common-extensions



sudo chown :i2c /dev/i2c*
sudo chmod g+rw /dev/i2c*
sudo usermod -aG i2c $USER

ssh-keygen -t ed25519
