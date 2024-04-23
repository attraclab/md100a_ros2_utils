#!/bin/bash

rosws=dev_ws
rospackage=md100a_ros2_utils

sleep 50

export DISPLAY=:0.0
export LOGFILE=/home/$USER/$rosws/src/$rospackage/autostart_scripts/cmd_vel_node.log

source /home/$USER/$rosws/src/$rospackage/autostart_scripts/ROS_CONFIG.txt
#source /opt/ros/noetic/setup.bash
source /opt/ros/galactic/setup.bash
source /home/$USER/$rosws/install/local_setup.bash


while true
do
		echo >>$LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		date >> $LOGFILE

		echo "Starting cmd_vel_converter node" >> $LOGFILE
		
		ros2 run md100a_ros2_utils cmd_vel_converter >> $LOGFILE

		echo "program seems to have stopped" >> $LOGFILE

		date >> $LOGFILE
		sleep 1

done
