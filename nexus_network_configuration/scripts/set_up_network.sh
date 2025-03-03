#!/bin/bash

# This script does the following:
# 1. Install RMW Zenoh (Jazzy dist.) if not already installed
# 2. Change RMW_IMPLEMENTATION to rmw_zenoh_cpp
# 3. Enable multicast on loopback interface

RMW_PACKAGE="ros-jazzy-rmw-zenoh-cpp"

PKG_OK=$(dpkg-query -W --showformat='${Status}\n' $RMW_PACKAGE|grep "install ok installed")

echo "Checking if $REQUIRED_PKG is installed: $PKG_OK"

if [ "$PKG_OK" = "" ]; then
  while true; do
      read -p "$RMW_PACKAGE not installed. Would you like to install it? (y/n)" yn
      case $yn in
          [Yy]* ) sudo apt-get --yes install $RMW_PACKAGE; break;;
          [Nn]* ) exit;;
          * ) echo "Please answer yes or no.";;
      esac
  done
fi

while true; do
    read -p "Restart ROS Daemon and set RMW_IMPLEMENTATION to 'rmw_zenoh_cpp'? (y/n)" yn
    case $yn in
        [Yy]* )
          ros2 daemon stop;
          echo "Stopped ROS2 Daemon"
          export RMW_IMPLEMENTATION=rmw_zenoh_cpp;
          ros2 daemon start;
          echo "Started ROS2 Daemon"
          break;;
        [Nn]* ) break;;
        * ) echo "Please answer yes or no.";;
    esac
done

while true; do
    read -p "Add route to enable multicast on loopback interface? (y/n)" yn
    case $yn in
        [Yy]* )
          # Enable multicast on loopback interface
          sudo ifconfig lo multicast;
          sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo;
          echo "Enabled multicast on loopback interface"
          break;;
        [Nn]* ) break;;
        * ) echo "Please answer yes or no.";;
    esac
done

while true; do
    read -p "Set ROS_LOCALHOST_ONLY to 1? (y/n)" yn
    case $yn in
        [Yy]* )
          # Export environment variables for configuring LOCALHOST only communication
          export ROS_LOCALHOST_ONLY=1;
          echo "ROS_LOCALHOST_ONLY set to 1"
          break;;
        [Nn]* ) exit;;
        * ) echo "Please answer yes or no.";;
    esac
done
