#!/bin/bash

rosws=dev_ws
rospackage=md100a_ros2_utils

sleep 10

export DISPLAY=:0.0
export LOGFILE=/home/$USER/$rosws/src/$rospackage/autostart_scripts/rosserial_node.log

source /home/$USER/$rosws/src/$rospackage/autostart_scripts/ROS_CONFIG.txt
source /opt/ros/noetic/setup.bash
#source /home/$USER/$rosws/install/setup.bash


while true
do
		echo >>$LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		date >> $LOGFILE

		echo "Starting rosserial" >> $LOGFILE
		
		rosrun rosserial_python serial_node.py  _port:=/dev/md100a _baud:=921600 >> $LOGFILE

		echo "program seems to have stopped" >> $LOGFILE

		date >> $LOGFILE
		sleep 1

done
