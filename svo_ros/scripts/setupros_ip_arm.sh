#!/bin/bash

# change the ip to your PC where you run the roscore
export ROS_MASTER_URI=http://192.168.200.7:11311

ip=$(ip addr | grep 'state UP' -A2 | tail -n1 | awk '{print $2}' | cut -f1  -d'/')

export ROS_IP=$ip
export ROS_HOSTNAME=$ip
