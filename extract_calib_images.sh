#!/bin/bash
# Script to extract image sequences from a rosbag file - specifically the tri-imager camera
#
#
# Before running this script make sure that the bagfile is placed in the "data/" folder inside
# this package.
# This script will also save calibration images to a folder inside the "data/" folder so please
# take that into consideration.
#
#

splitter='/';
echo "Rosbag filename: " $1

bagname=$PWD$splitter$1;
savename=$PWD$splitter$2;

if [ ! -d "$savename" ]; then
	echo "Directory does not exist. Creating directory.."
	# create a folder for the images
	mkdir "$savename"
	# call the export launch file and pass source and destination parameters
	roslaunch export.launch bag_name:=$bagname save_name:=$savename
else
	echo "Image directory already exists. Cannot automatically overwrite."
fi


