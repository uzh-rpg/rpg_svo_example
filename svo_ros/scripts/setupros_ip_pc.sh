#!/bin/bash

# if you run everything locally
# ip=127.0.0.1
# run on multiple platforms
ip=$(ip addr | grep 'state UP' -A2 | tail -n1 | awk '{print $2}' | cut -f1  -d'/')

core_ip=$ip

export ROS_IP=$ip
export ROS_HOSTNAME=$ip
export ROS_MASTER_URI=http://${core_ip}:11311

export ROSLAUNCH_SSH_UNKNOWN=1
