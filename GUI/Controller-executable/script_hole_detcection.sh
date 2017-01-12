#!/bin/sh
 
#dir=`find / -name $2 2>/dev/null`
echo $2
cd $2
source devel/setup.bash
rostopic pub /detect_cmd bolt_detection/Detection  "detect: true" -1
