#!/bin/bash
DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )
# install KOMO dependencies
$DIR/KOMO/install/INSTALL_ALL_UBUNTU_PACKAGES.sh

# install ros_control packages, necessary for iis_control package
sudo apt-get install ros-indigo-ros-control ros-indigo-ros-controllers
