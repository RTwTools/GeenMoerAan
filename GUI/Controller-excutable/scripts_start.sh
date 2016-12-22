#!/bin/bash
 
dir=`find / -name $1 2>/dev/null`
echo $dir
cd $dir
source devel/setup.bash
xterm -hold -e ' roslaunch robot_mover start_system.launch && bash ' &
