#!/bin/bash
 
#dir=`find / -name $1 2>/dev/null`
cd $1
pwd
source devel/setup.bash
roslaunch robot_mover start_system.launch & 
