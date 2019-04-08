#!/bin/bash
calib_dir="/home/nvidia/ros/src/fla_estimation/packages/fla_calibration/calib/"
l_intrinsic_file="left_intrinsics_camerainfo.yaml"
r_intrinsic_file="right_intrinsics_camerainfo.yaml"
left_intr=$calib_dir$l_intrinsic_file
right_intr=$calib_dir$r_intrinsic_file
echo $left_intr
echo $right_intr
extr_file="ovc-astra.yaml"
extrinsic_file=$calib_dir$extr_file
echo $extrinsic_file
output_dir="/home/nvidia/ros/src/cuda-sgm-ros/other/"
output_file="stereo_calibration_sgm.yml"
output_location=$output_dir$output_file
echo $output_location
./convert $left_intr $right_intr $extrinsic_file $output_location 1280 1024

corner_fname="corners.csv"
# Copy corners.csv as backup
output_destination=$output_dir$corner_fname
echo $output_destination
cp ~/.ros/corners.csv $output_destination
