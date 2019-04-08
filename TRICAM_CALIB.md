# Stereo Camera Calibration with Dual Baseline for use with OSRF Falcam v1.0 and NVIDIA TX1/TX2

##### Requirements:
1. ROS Kinetic
2. Opencv 2.4.X
3. OSRF Falcam or similar camera (two baselines)
4. popt 

##### Usage:

Input: Rosbag with concatenated image
Output: Camera Parameters for Camera 1 (C1), Camera 2 (C2) and Camera 3 (C3) and Extrinsics for Baseline C1_2 (narrow) and C1_3 (wide)

Before running this script make sure that the bagfile is placed in the "data/" folder inside this package. This script will also save calibration images to a folder inside the "data/" folder so please take that into consideration. If data folder does not exist already, create a data folder in the project directory.

```
sudo apt-get install libpopt-dev
cd ~/path/to/tegra_tricam_calib
make -j2
mkdir data
```

Run the script that extracts images from the rosbag

```
./extract_calib_images.sh data/ros_bag_file_name.bag data/folder_for_images
```

This will create a folder with all the calibration images, you can then run the calibration script with
the follwoing parameters,

```
./calibrate -w 9 / (number of calibration squares along horizontal direction)
            -h 6 / (number of calibration squares along vertical direction)
            -s 0.27 / (dimension of each calibration square)
            -d data/folder_for_images/ / (destination of image folder previously created)
            -c calibration_narrow.yaml / (location and name for narrow baseline calibration parameters)
            -f calibration_wide.yaml / (location and name for wide baseline calibration parameters)
            -v 1 / (verbosity flag: 1 will display intermediate images and detected points)
```

In <b>calibrate.cpp</b> you can also specify parameters for an initial offset skip, end offset skip as
well as a frame skip count.

With the parameters obtained in this script you can test it on a rosbag using the <b>rectify.cpp</b> script  provided in the same repository.

If you have any questions, feel free to raise an issue or contact me privately.




