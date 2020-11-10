### Setup Raspbian buster (10) aarch64 on Raspberry Pi 4

*/boot/config.txt*
```
#uncomment to overclock the arm. 1500 MHz is the default.
over_voltage=4
arm_freq=2000

# Boot optimisations
disable_splash=1
dtoverlay=sdtweak,overclock_50=100
dtoverlay=pi3-disable-bt

# Uncomment some or all of these to enable the optional hardware interfaces
dtparam=i2c_arm=on,i2c_arm_baudrate=400000
dtoverlay=i2c6,baudrate=400000
dtoverlay=i2c5,baudrate=100000
dtoverlay=i2c4,baudrate=400000
dtoverlay=i2c3,baudrate=400000
dtoverlay=i2c1,baudrate=400000
#dtparam=i2s=on
#dtparam=spi=on

# Additional overlays and parameters are documented /boot/overlays/README

# Enable audio (loads snd_bcm2835)
dtparam=audio=off

# Enable DRM VC4 V3D driver on top of the dispmanx display stack
dtoverlay=vc4-fkms-v3d
max_framebuffers=2
arm_64bit=1
enable_uart=1
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
