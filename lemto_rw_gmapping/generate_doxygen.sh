#!/bin/bash          
#
# Script to generate doxygen documentation
#
# Koen Lekkerkerker
# 17 Jun 2014 
#
# Usage:
# ./generate_doxygen.sh
#
# Output will end in the "doc" subfolder of the current package
# Open doc/html/index.html to view the documentation

cd "${0%/*}" #make current working directory the folder of this package

pkg_name=${PWD##*/} # extract the foldername from the current working directory

rosdoc_lite $(rospack find $pkg_name) -o $(rospack find $pkg_name)/doc
