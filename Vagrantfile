# -*- mode: ruby -*-
# vi: set ft=ruby :

# All Vagrant configuration is done below. The "2" in Vagrant.configure
# configures the configuration version (we support older styles for
# backwards compatibility). Please don't change it unless you know what
# you're doing.
Vagrant.configure("2") do |config|
  # The most common configuration options are documented and commented below.
  # For a complete reference, please see the online documentation at
  # https://docs.vagrantup.com.

  # Every Vagrant development environment requires a box. You can search for
  # boxes at https://vagrantcloud.com/search.
  config.vm.box = "ubuntu/bionic64"

  # Disable automatic box update checking. If you disable this, then
  # boxes will only be checked for updates when the user runs
  # `vagrant box outdated`. This is not recommended.
  # config.vm.box_check_update = false

  # Create a forwarded port mapping which allows access to a specific port
  # within the machine from a port on the host machine. In the example below,
  # accessing "localhost:8080" will access port 80 on the guest machine.
  # NOTE: This will enable public access to the opened port
  config.vm.network "forwarded_port", guest: 8080, host: 8080

  # Create a forwarded port mapping which allows access to a specific port
  # within the machine from a port on the host machine and only allow access
  # via 127.0.0.1 to disable public access
  # config.vm.network "forwarded_port", guest: 80, host: 8080, host_ip: "127.0.0.1"

  # Create a private network, which allows host-only access to the machine
  # using a specific IP.
  # config.vm.network "private_network", ip: "192.168.33.10"

  # Create a public network, which generally matched to bridged network.
  # Bridged networks make the machine appear as another physical device on
  # your network.
  config.vm.network "public_network"

  # Share an additional folder to the guest VM. The first argument is
  # the path on the host to the actual folder. The second argument is
  # the path on the guest to mount the folder. And the optional third
  # argument is a set of non-required options.
  config.vm.synced_folder ".", "/home/vagrant/ros"

  # Provider-specific configuration so you can fine-tune various
  # backing providers for Vagrant. These expose provider-specific options.
  # Example for VirtualBox:
  #
  config.vm.provider "virtualbox" do |vb|
    # Display the VirtualBox GUI when booting the machine
    vb.gui = false

    # Number of Logical CPUs
    vb.cpus = 2

    # Customize the amount of memory on the VM:
    vb.memory = "3072"
  end
  #
  # View the documentation for the provider you are using for more
  # information on available options.

  # Enable provisioning with a shell script. Additional provisioners such as
  # Puppet, Chef, Ansible, Salt, and Docker are also available. Please see the
  # documentation for more information about their specific syntax and use.
  config.vm.provision "shell", inline: <<-SHELL
    sudo apt-get update && sudo apt-get upgrade -y
    echo "Installing ROS 2 Eloquent \n\n"
    echo "Setting up Locale \n"
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    echo "Setting up Sources \n"
    sudo apt-get install -y curl gnupg2 lsb-release ntp
    curl http://repo.ros2.org/repos.key | sudo apt key add -
    sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

    export ROS_DISTRO=eloquent
    echo "Installing ROS 2 $ROS_DISTRO packages\n"
    sudo apt-get update
    sudo apt-get install -y ros-$ROS_DISTRO-ros-base ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-xacro ros-$ROS_DISTRO-py-trees ros-$ROS_DISTRO-py-trees-ros libopencv-dev libboost-dev

    echo "Installing optional argcomplete \n"
    sudo apt-get install -y python3-argcomplete python3-colcon-common-extensions python3-pip
    python3 -m pip install flask flask-socketio
    . /opt/ros/$ROS_DISTRO/setup.bash
    echo ". /opt/ros/$ROS_DISTRO/setup.bash" >> /home/vagrant/.bashrc

    echo "Installing ROS 2 navigation from master branch"
    git clone https://github.com/ros-planning/navigation2.git /home/vagrant/ros_ws/navigation2
    cd /home/vagrant/ros_ws/
    colcon build --symlink-install --packages-skip nav2_system_tests


  SHELL
end
