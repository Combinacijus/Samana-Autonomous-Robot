#!/bin/bash
echo "Creating soft link from ros_lib_* to my default Arduino libraries"

lib_path_samana="/home/combinacijus/Documents/SamanaAutonomousRobot/Arduino/libraries/"
lib_path_arduino="/home/combinacijus/Documents/Arduino/libraries/"

ln -sv "${lib_path_samana}ros_lib/" 			"$lib_path_arduino"
ln -sv "${lib_path_samana}ros_lib_sonar/" 		"$lib_path_arduino"
ln -sv "${lib_path_samana}ros_lib_imu/" 		"$lib_path_arduino"
ln -sv "${lib_path_samana}ros_lib_motor/" 		"$lib_path_arduino"
ln -sv "${lib_path_samana}ros_lib_gps_odom/" 	"$lib_path_arduino"
ln -sv "${lib_path_samana}ros_lib_arm/" 		"$lib_path_arduino"