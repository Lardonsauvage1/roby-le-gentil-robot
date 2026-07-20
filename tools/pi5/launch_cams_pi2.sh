#!/bin/bash
# Lance les 2 cameras via cam_pub_pi2.py (picamera2, exposition/gain/WB FIGES = reproductible).
# gauche (exterieure) = i2c@80000 ; droite (poignet) = i2c@88000.
LC=/home/roby/lc_src/libcamera-0.5.2+rpt20250903/build/src/py
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///home/roby/cyclone_config.xml
# /usr/lib en tete => bindings py3.12 utilisent la libcamera systeme (PiSP) ; garde ROS
export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH
export PYTHONPATH=/home/roby/pystubs:$LC:$PYTHONPATH

# Valeurs FIGEES (relevees de l'auto le 2026-07-16, a re-tuner si besoin).
# left = EXTERIEURE = i2c@88000 (neutre, bien exposee) ; right = POIGNET = i2c@80000.
python3 /home/roby/cam_pub_pi2.py --side left  --cam i2c@88000 --fps 15 --rot180 \
  --exposure 66640 --gain 6.875 --red-gain 1.039 --blue-gain 1.616 \
  > /home/roby/pi2_left.log  2>&1 &
python3 /home/roby/cam_pub_pi2.py --side right --cam i2c@80000 --fps 15 \
  --exposure 66640 --gain 8.0   --red-gain 1.047 --blue-gain 1.877 \
  > /home/roby/pi2_right.log 2>&1 &
wait
