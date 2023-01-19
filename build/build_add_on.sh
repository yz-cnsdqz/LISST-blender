#!/bin/bash
pushd ..

filedate=$(date '+%Y%m%d')

# Build 10 shape model add-on
#default
archivename=./lisst_blender_addon_$filedate.zip

case "$OSTYPE" in
  solaris*) echo "SOLARIS" ;;
  darwin*)  echo "OSX" ;; 
  linux*)   echo "LINUX" ;;
  bsd*)     echo "BSD" ;;
  msys*)    
    echo "WINDOWS" 
    archivename=./LISST-blender/lisst_blender_addon_$filedate.zip
    ;;
  *)        echo "unknown: $OSTYPE" ;;
esac

echo "Generating $archivename"

if [ -f $archivename ]; then
  echo "Removing old add-on: $archivename"
  rm $archivename
fi

zip -r "$archivename" LISST-blender/*.py LISST-blender/*.md LISST-blender/data/*.fbx LISST-blender/data/*.pkl #LISST-blender/data/*.png 

popd
