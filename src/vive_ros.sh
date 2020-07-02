#!/bin/bash

# Runs the two required launch files for running the Vive in ROS
# (server_vr.launch and vive.launch) and then runs the script
# 'close_servervr.sh' when CTRL-C is pressed. This allows the Vive to be opened
# and closed by running a single command.

function close_server()
{
    rosrun vive_ros close_servervr.sh
}

trap close_server SIGINT EXIT

roslaunch vive_ros server_vr.launch &

sleep 2

roslaunch vive_ros vive.launch
