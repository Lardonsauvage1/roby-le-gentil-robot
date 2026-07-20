#!/bin/bash
LC=/home/roby/lc_src/libcamera-0.5.2+rpt20250903/build/src/py
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI=file:///home/roby/cyclone_config.xml
export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH
export PYTHONPATH=/home/roby/pystubs:$LC:$PYTHONPATH
python3 /home/roby/cam_pub_pi2.py --side right --cam i2c@80000 --fps 15 --exposure 8000 --gain 4.0 --red-gain 1.25 --blue-gain 2.7 > /home/roby/pi2_solo.log 2>&1 &
wait
