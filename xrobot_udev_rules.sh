#!/bin/bash

echo ""
echo "This script copies a udev rule to /etc"
echo ""

script_dir=$(cd $(dirname $0);pwd)

sudo systemctl stop nvgetty
sudo systemctl disable nvgetty
sudo cp ${script_dir}/56-orbbec-usb.rules /etc/udev/rules.d
sudo cp ${script_dir}/20-serial.rules /etc/udev/rules.d


echo ""
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart