#!/bin/bash          
#
# Script to supress gmapping output
#
# Koen Lekkerkerker
# 7 May 2014 
#
#

echo
echo 
echo "************************* WARNING ************************"
echo "YOU ARE CURRENTLY USING GMAPPING ASSUMING PERFECT ODOMETRY"
echo "************************* WARNING ************************"
echo 
echo

roslaunch st_navigation gmapping_perfectodometry.launch scan_topic:=$1 2>1
