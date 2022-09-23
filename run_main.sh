#!/bin/bash

cd ~/LIO_SAM
chmod u+r+x run_roscore.sh
chmod u+r+x run_imu_calib.sh
chmod u+r+x run_imu_converter.sh
chmod u+r+x run_vlp16.sh
chmod u+r+x run_lio_sam.sh
#./run_roscore.sh && ./run_imu_calib.sh && ./run_imu_converter.sh && ./run_vlp16.sh && ./run_lio_sam.sh
./run_imu_calib.sh #&& ./run_imu_converter.sh #&& ./run_vlp16.sh && ./run_lio_sam.sh
