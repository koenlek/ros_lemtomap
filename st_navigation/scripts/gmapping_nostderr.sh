#!/bin/bash          
#
# Script to supress gmapping output
#
# Koen Lekkerkerker
# 7 May 2014 
#
#

roslaunch st_navigation gmapping.launch scan_topic:=$1 2>1
