#!/bin/bash

rosws=dev_ws
rospackage=md100a_ros2_utils

sleep 25

export DISPLAY=:0.0
export LOGFILE=/home/$USER/$rosws/src/$rospackage/autostart_scripts/ros1_bridge_node.log

source /home/$USER/$rosws/src/$rospackage/autostart_scripts/ROS_CONFIG.txt
source /opt/ros/noetic/setup.bash
source /opt/ros/galactic/setup.bash
source /home/$USER/$rosws/install/local_setup.bash


while true
do
		echo >>$LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		date >> $LOGFILE

		echo "Starting ros1_bridge" >> $LOGFILE
		
		ros2 run ros1_bridge dynamic_bridge >> $LOGFILE

		echo "program seems to have stopped" >> $LOGFILE

		date >> $LOGFILE
		sleep 1

done
