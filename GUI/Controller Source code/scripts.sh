#!/bin/bash
 
dir=`find / -name Robot 2>/dev/null`
cd $dir
source devel/setup.bash
xterm -e ' roslaunch robot_mover robot_mover.launch && bash ' &
detect=`find / -name Bolt_detection 2>/dev/null`
cd $detect
source devel/setup.bash
xterm -hold -e '  roslaunch bolt_detection bolt_detector.launch gui:=true ' &

#xterm -hold -e bash -i -c 'roslaunch abb_irb120_moveit_config moveit_planning_execution.launch' &

# xterm -hold -e bash -i -c   \'").append("var=`find / -name Ass5`  && echo $var &" ).append("\'")
