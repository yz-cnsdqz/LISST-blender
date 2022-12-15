#!/bin/bash
pushd ..

filedate=$(date '+%Y%m%d')

# Build 10 shape model add-on
archivename=./lisst_blender_addon_$filedate.zip
echo "Generating $archivename"

if [ -f $archivename ]; then
  echo "Removing old add-on: $archivename"
  rm $archivename
fi

zip -r "C:/LISST-blender/$archivename" LISST-blender/*.py LISST-blender/*.md LISST-blender/data/*.fbx LISST-blender/data/*.pkl #LISST-blender/data/*.png 

popd