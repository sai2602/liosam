#!/bin/bash

liosam=$PWD
cd $PWD/tools/ros_converter_imu/ros_topic_converter/catkin_ws
source devel/setup.bash
rosrun IMU_RAW IMU_RAW_node _setting:=read_sol _csv_path:=$liosam/PAD_csv/PAD_solution.bin.csv
