#!/usr/bin/env bash

ssh-keygen -t ed25519

apt-get update && apt upgrade -y
apt-get install -y qemu-user-static docker.io python3-pip

pip3 install ros_cross_compile vcstool

mkdir -p ~/ros_ws/src && cd ~/ros_ws/src
git clone git@github.com:3wnbr1/ros.git
vcs import < ./ros/ros_ws.repos

ros_cross_compile ~/ros_ws --arch aarch64 --os ubuntu --rosdistro foxy
