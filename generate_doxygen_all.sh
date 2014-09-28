#!/bin/bash          
#
# Script to generate doxygen documentation for all packages
#
# Koen Lekkerkerker
# 17 Jun 2014 
#
# Usage:
# ./generate_doxygen_all.sh
#
# Output will end in the "doc" subfolder of each package
# Open doc/html/index.html to view the documentation

cd "${0%/*}" #make current working directory the folder of this package

for i in $(find . -name 'generate_doxygen.sh') #store an array of all paths to a generate_doxygen.sh, that can be found from the parent folder of the lemtomap metapackage
	do ./$i #execute these generate_doxygen.sh scripts
done
