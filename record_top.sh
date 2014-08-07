#!/bin/bash          
#
# Script to record top, to postprocess later in Matlab for benchmarks (CPU and mem usage)
#
# Koen Lekkerkerker
# Created: 	9 Jul 2014 
# Last updated: 9 Jul 2014
#
# Usage:
# Manually specify the processes to record in this file
# ./record_top.sh
#
# Todo:
# - Maybe limit load/writing/file size by limiting the size of the header. This can be done through a personal top configuration file. Maybe even add one initial top run with all header, and then only with the time line for example...
#   - could also be used to sort by PID
# - Maybe switch to htop or ps (easily select columns from command line, non-truncated process name)
# - Memory accuracy/units:
#	- (works for 14.04) When in top, typing "E" cycles through different memory units (kb, mb, gb etc) in the total memory info.
# - Full process names

########### SETTINGS #############
CURRENT_DATE=$(date +%F_%R:%S)
STORE_PATH=../../benchmarks/top_measurement_$CURRENT_DATE.txt

PROCESSES=(
#"slam_gmapping"
"st_gmapping_rolling"
"move_base"
"move_base_topo"
"topological_navigation_mapper"
) #max 20 processes! This is a limitation of top
TIME_STEP=00.20

########### PROCESSING #############

cd "${0%/*}" #make current working directory the folder of this package

PIDS=""
PIDS_PRINT=""

for i in "${PROCESSES[@]}"
do
   PIDS="$(pidof $i), $PIDS"
   PIDS_PRINT="$PIDS_PRINT$i $(pidof $i)\n"
done

PIDS=${PIDS:0:-2} #removes the last comma

printf "$PIDS_PRINT" > $STORE_PATH
printf "StepSize: $TIME_STEP\n" >> $STORE_PATH
printf "WallTime: %f\n\n" $(date +%s.%N) >> $STORE_PATH
top -b -p$PIDS -d $TIME_STEP >> $STORE_PATH
# > will write to file, >> wil append to file!
# -b sets batch mode with updates every -d seconds
# -p supplies the pids
# -S turns on comulative time mode ->  Starts  top  with the last remembered 'S' state reversed.  When 'Cumulative mode' is On, each process is listed with the cpu time that it and its dead children have used.  See the 'S' interactive command for additional information regarding this mode.
