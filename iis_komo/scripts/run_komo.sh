#!/bin/bash

# ensure tmp directory and MT.cfg
KOMO_DIR=`rospack find iis_komo`
if [ ! -d $KOMO_DIR/tmp ]
  then
    mkdir $KOMO_DIR/tmp
    ln -s $KOMO_DIR/config/MT.cfg $KOMO_DIR/tmp
fi


# check input argument
if [ -z "$1" ]
  then
    ROS_NS="simulation"
  else 
    ROS_NS=$1
fi

echo "Launching iis_komo in namespace '$ROS_NS'"

# iis_komo only works when working directory is correctly set
# this directory needs to contain MT.cfg (file or symlink)
cd `rospack find iis_komo`/tmp
# set namespace
export ROS_NAMESPACE=$ROS_NS
# start iis_komo node
rosrun iis_komo iis_komo
