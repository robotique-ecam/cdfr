### Setup Ubuntu 20.04.1 LTS on Raspberry Pi 4

*/boot/firmware/syscfg.txt*
```
# This file is intended to be modified by the pibootctl utility. User
# configuration changes should be placed in "usercfg.txt". Please refer to the
# README file for a description of the various configuration files on the boot
# partition.

enable_uart=0
dtparam=audio=off
dtparam=i2c_arm=on
dtparam=spi=off
cmdline=cmdline.txt
```

*/boot/firmware/usercfg.txt*
```
# Place "config.txt" changes (dtparam, dtoverlay, disable_overscan, etc.) in
# this file. Please refer to the README file for a description of the various
# configuration files on the boot partition.
dtparam=i2c_arm=on,i2c_arm_baudrate=10000
dtoverlay=i2c6,baudrate=400000
dtoverlay=i2c5,baudrate=10000
dtoverlay=i2c4,baudrate=400000
dtoverlay=i2c3,baudrate=400000
dtoverlay=i2c1,baudrate=10000
dtoverlay=i2c0,baudrate=10000
```

*/etc/systemd/system/rc-local.service*
```
[Unit]
 Description=/etc/rc.local Compatibility
 ConditionPathExists=/etc/rc.local

[Service]
 Type=forking
 ExecStart=/etc/rc.local start
 TimeoutSec=0
 StandardOutput=tty
 RemainAfterExit=yes
 SysVStartPriority=99

[Install]
 WantedBy=multi-user.target
```

*/etc/rc.local*
```bash
#!/bin/bash

source /home/ubuntu/ros/install/setup.bash
HOME=/home/ubuntu /opt/ros/foxy/bin/ros2 launch asterix launch.py > /home/ubuntu/ros2-system-$(date "+%Y.%m.%d-%H.%M.%S").log 2>&1 &
```

*~/.bashrc*
```bash
...

source /home/ubuntu/ros/install/setup.bash
```

*/usr/bin/vcgencmd*
```
#!/bin/bash
sudo LD_LIBRARY_PATH=/opt/vc/lib /opt/vc/bin/vcgencmd "$@"
```

And run
```bash
sudo chmod +x /etc/rc.local
sudo systemctl enable rc-local

# Install vcgencmd
git clone --depth 1 https://github.com/raspberrypi/userland rpi-userland
cd rpi-userland && ./buildme --aarch64
sudo rm -rf rpi-userland

# Set SUID byte to run it with owner privileges
sudo chown root: /usr/bin/vcgencmd
sudo chmod 4755 /usr/bin/vcgencmd
```
