#!/bin/bash
# Lance la simulation MoveIt du bras neuroneimitationcarote
# Utilise CycloneDDS en mode local (pas besoin du réseau multi-machines)

# Config CycloneDDS locale (autodetermine l'interface réseau)
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface autodetermine=\"true\"/></Interfaces></General></Domain></CycloneDDS>"
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Nettoyer GTK_PATH snap qui casse rviz2
unset GTK_PATH

source /opt/ros/jazzy/setup.bash
source /home/sam/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42

echo "=== Lancement simulation neuroneimitationcarote ==="
echo "  RMW: CycloneDDS (local)"
echo "  Domain ID: $ROS_DOMAIN_ID"
echo ""
echo "  Utilisation dans RViz:"
echo "    - Déplacer les marqueurs orange pour choisir la pose cible"
echo "    - Panneau MotionPlanning > Planning > Plan & Execute"
echo ""

ros2 launch neuroneimitationcarote_moveit_config demo.launch.py
