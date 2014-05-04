#!/bin/bash
ifconfig eth0 192.168.10.1
echo BB-UART4 > /sys/devices/bone_capemgr.*/slots
#cd ~/RFGeolocation/BeagleBone/
#make clean
#make
sleep 30
./home/root/RFGeolocation/BeagleBone/Client
python /home/root/RFGeolocation/BeagleBone/SDRdata.py
