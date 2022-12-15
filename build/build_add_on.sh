#!/bin/bash
pushd ..

filedate=$(date '+%Y%m%d')

# Build 10 shape model add-on
echo "Generating $archivename"
archivename=./lisst_blender_addon_$filedate.zip
if [ -f $archivename ]; then
  echo "Removing old add-on: $archivename"
  rm $archivename
fi
zip $archivename LISST-blender/*.py LISST-blender/*.md LISST-blender/data/*.fbx LISST-blender/data/*.pkl LISST-blender/data/*.png 

popd