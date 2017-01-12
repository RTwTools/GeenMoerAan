#!/bin/bash
 
#dir=`find / -name $2 2>/dev/null`
echo $2
cd $2
source devel/setup.bash
var='rostopic pub  /detect_cmd bolt_detection/Detection  "detect:'$1
var=$var'" -1'
echo $var
xterm -hold -e $var &

