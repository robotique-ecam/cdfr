#!/bin/bash

sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt update && sudo apt install -y curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-setuptools \
  python3-vcstool \
  python3-argcomplete \
  python3-psutil \
  python3-rosdep \
  python3-pykdl \
  python3-smbus \
  python3-rpi.gpio \
  wget \
  i2c-tools \
  libi2c-dev \
  libasio-dev \
  libtinyxml2-dev

sudo -H pip3 install \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
  emrichen \
  hiyapyco \
  vcgencmd \
  vl53l1x

mkdir -p ~/ros2_foxy/src
cd ~/ros2_foxy
wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos
vcs import src < ros2.repos


sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro foxy -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers"


cd ~/ros2_foxy/
colcon build --symlink-install --cmake-args "-DBUILD_TESTING=OFF" "-DCMAKE_BUILD_TYPE=Release"
source ros2_foxy/install/setup.bash


mkdir -p ~/ros_ws/src
cd ~/ros_ws
vcs import src < ros_ws.repos
rosdep install --from-paths src --ignore-src --rosdistro foxy -y
colcon build --symlink-install --cmake-args "-DBUILD_TESTING=OFF" "-DCMAKE_BUILD_TYPE=Release"
source ros_ws/install/setup.bash


cd ~
git clone --recursive git@github.com:3wnbr1/ros.git
