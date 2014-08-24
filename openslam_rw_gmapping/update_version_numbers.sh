#!/bin/bash          
#
# Script to update all package versions (in package.xml) for the semantic turtle stack
#
# Koen Lekkerkerker
# Created: 	8 Jul 2014 
# Last updated: 8 Jul 2014
#
# Usage:
# ./update_version_numbers.sh [version]
#
# Example:
# ./update_version_numbers.sh 0.2.1
#

cd "${0%/*}" #make current working directory the folder of this package

if [ "$1" = "" ]; then
  echo ""
  echo "please pass a version number, e.g. do ./update_version_numbers.sh 0.2.1-patch1"
  echo ""
  exit 0
fi

OLD="<version>.*</version>"
NEW="<version>$1</version>"

OLD="${OLD//\//\\/}" #substitue any slashes
NEW="${NEW//\//\\/}"

for f in $(find . -name 'package.xml') #store an array of all paths to a package.xml, that can be found from the parent folder of the semantic turtle metapackage 
	do
	  echo "Updating $f to version $1"
	  sed "s/${OLD}/${NEW}/g" "$f" > "$f.bak" && mv "$f.bak" "$f"
done
