#!/bin/bash

rosws=dev_ws
rospackage=md100a_ros2_utils

sleep 5

export DISPLAY=:0.0
export LOGFILE=/home/$USER/$rosws/src/$rospackage/autostart_scripts/roscore_node.log

source /home/$USER/$rosws/src/$rospackage/autostart_scripts/ROS_CONFIG.txt
source /opt/ros/noetic/setup.bash
#source /home/$USER/$rosws/install/setup.bash


while true
do
		echo >>$LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		date >> $LOGFILE

		echo "Starting roscore" >> $LOGFILE
		
		roscore >> $LOGFILE

		echo "program seems to have stopped" >> $LOGFILE

		date >> $LOGFILE
		sleep 1

done
