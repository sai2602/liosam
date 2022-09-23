#!/bin/bash

cd $PWD/tools/velodyne_driver/catkin_ws
source ./devel/setup.bash
roslaunch velodyne_pointcloud VLP16_points.launch
