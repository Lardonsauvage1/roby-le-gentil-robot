# A SOURCER dans chaque terminal PC pour la session calibration :  source ~/roby_env.sh
# Neutralise pyenv (3.11 sans tkinter/rclpy) et met l'env ROS Jazzy + DDS correct.
unset PYENV_VERSION 2>/dev/null
unset VIRTUAL_ENV 2>/dev/null
source /opt/ros/jazzy/setup.bash
# prefixe /usr/bin pour que python3 = systeme 3.12 (et non pyenv/venv/linuxbrew)
export PATH="/opt/ros/jazzy/bin:/usr/bin:/bin:$PATH"
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/cyclone_config.xml"
unset GTK_PATH 2>/dev/null
echo "[roby_env] ROS Jazzy + DDS domain 42 (enp86s0 -> Pi5 192.168.2.37), pyenv neutralise."
