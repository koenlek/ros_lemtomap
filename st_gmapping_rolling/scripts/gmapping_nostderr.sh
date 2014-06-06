#!/bin/bash          
#
# Script to supress gmapping output
#
# Koen Lekkerkerker
# 7 May 2014 
#
# Usage:
# gmapping_nostderr.sh <suppress> <launchfile> <arguments for launch file>
#
# <suppress>: if suppress is set to 0, it will not suppres!
# <launchfile>: the launchfile should be a file that is within the st_gmapping_rolling folder
# <arguments for launch file>: optionally, you can pass arguments to that launch file
#

if [ "$1" = "0" ]; then
 shift
 roslaunch st_gmapping_rolling $@
else
 shift
 echo "now running 'roslaunch st_gmapping_rolling $@'"
 roslaunch st_gmapping_rolling $@ 2>1
fi
