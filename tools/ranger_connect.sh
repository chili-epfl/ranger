#!/bin/bash 

echo -e "*** Configuring bluetooth ***"
hcitool scan
sudo rfcomm bind 0 10:00:E8:6C:F0:4F

echo -e "\n*** Starting AsebaSwitch ***"
cd /home/david/aseba/build-aseba/switches/switch
sudo ./asebaswitch "ser:device=/dev/rfcomm0;fc=hard;baud=921600"
