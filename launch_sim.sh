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
# Domaine 43, JAMAIS 42 : 42 est celui de la vraie stack (Pi5). Ce launch demarre un robot
# SIMULE (mock ros2_control + robot_state_publisher) ; sur 42 il publierait un second
# /robot_description et un faux /joint_states a cote du vrai bras (course « mock » decrite
# dans /roby-lancer-bras, BUG-008). Il utilisait 42 jusqu'au 2026-09-13.
export ROS_DOMAIN_ID="${ROBY_SIM_DOMAIN_ID:-43}"
if [ "$ROS_DOMAIN_ID" = "42" ]; then
    echo "REFUS : domaine 42 = vraie stack. Simuler sur un autre domaine." >&2
    exit 1
fi

echo "=== Lancement simulation neuroneimitationcarote ==="
echo "  RMW: CycloneDDS (local)"
echo "  Domain ID: $ROS_DOMAIN_ID"
echo ""
echo "  Utilisation dans RViz:"
echo "    - Déplacer les marqueurs orange pour choisir la pose cible"
echo "    - Panneau MotionPlanning > Planning > Plan & Execute"
echo ""

ros2 launch neuroneimitationcarote_moveit_config demo.launch.py
