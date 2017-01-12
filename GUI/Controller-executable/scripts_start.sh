#!/bin/bash
 
#dir=`find / -name $1 2>/dev/null`
echo $1
cd $1
source devel/setup.bash
roslaunch robot_mover start_system.launch & 
