#!/bin/bash
 
#dir=`find / -name $2 2>/dev/null`
cd $2
pwd
source devel/setup.bash
rostopic pub /detect_cmd bolt_detection/Detection  "detect: true" -1
