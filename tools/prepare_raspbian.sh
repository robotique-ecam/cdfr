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
  python3-serial \
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

sudo systemctl disable dphys-swapfile.service
sudo systemctl disable keyboard-setup.service
sudo systemctl disable apt-daily.service
sudo systemctl disable apt-daily-upgrade.service
sudo systemctl disable hciuart.service
sudo systemctl disable raspi-config.service
sudo systemctl disable avahi-daemon.service
sudo systemctl disable triggerhappy.service

sudo systemctl mask dphys-swapfile.service
sudo systemctl mask keyboard-setup.service
sudo systemctl mask apt-daily.service
sudo systemctl mask apt-daily-upgrade.service
sudo systemctl mask hciuart.service
sudo systemctl mask raspi-config.service
sudo systemctl mask avahi-daemon.service
sudo systemctl mask triggerhappy.service

if [[ $BUILD = "ON" ]]; then
  mkdir -p ~/ros2_foxy/src
  cd ~/ros2_foxy
  wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos
  vcs import src < ros2.repos

  sudo mkdir -p /opt/ros/foxy/install
  sudo chown -R pi: /opt/ros

  export CFLAGS="-march=native"
  export CXXFLAGS=${CFLAGS}

  sudo rosdep init
  rosdep update
  rosdep install --from-paths src --ignore-src --rosdistro foxy -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers"

  cd ~/ros2_foxy/
  colcon build --install-base /opt/ros/foxy/install --merge-install --cmake-args "-DBUILD_TESTING=OFF" "-DCMAKE_BUILD_TYPE=Release"
  source /opt/ros/foxy/install/setup.bash

  mkdir -p ~/ros_ws/src
  cd ~/ros_ws
  vcs import src < ros_ws.repos
  rosdep install --from-paths src --ignore-src --rosdistro foxy -y --skip-keys "gazebo_ros_pkgs slam_toolbox"
  colcon build --install-base /opt/ros/foxy/install --merge-install --cmake-args "-DBUILD_TESTING=OFF" "-DCMAKE_BUILD_TYPE=Release" "-DCMAKE_CXX_STANDARD_LIBRARIES=-lpython3.7m" --packages-skip "nav2_system_tests"
  source /opt/ros/foxy/install/setup.bash
else

fi

cd ~
git clone --recursive git@github.com:3wnbr1/ros.git
