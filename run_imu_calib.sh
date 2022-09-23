#!/bin/bash

echo "Starting imu calib"

cd $PWD/tools/imu_calibration/catkin_ws
source devel/setup.bash
./devel/lib/imu_calib/apply_calib
