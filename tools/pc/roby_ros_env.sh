# Environnement ROS du PC — UNE seule copie. A SOURCER :  source roby_ros_env.sh
#
# ROBY_SIM=1  => simulation : domaine 43, JAMAIS celui du vrai bras
# sinon       => vrai robot : domaine 42
#
# Le workspace est celui qui CONTIENT ce fichier (lien du home resolu) : un script de tools/pc
# utilise toujours les paquets compiles a cote de lui, jamais ceux d'une autre copie du depot.
# Pas de `set -u` : les setup.bash de ROS lisent des variables non definies.

_roby_ici="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
ROBY_WS="$(cd "$_roby_ici/../.." && pwd)"
export ROBY_WS
unset _roby_ici

if [ ! -f "$ROBY_WS/install/setup.bash" ]; then
    echo "roby_ros_env : $ROBY_WS/install/setup.bash absent (colcon build dans $ROBY_WS)." >&2
    return 1 2>/dev/null || exit 1
fi

# pyenv/venv : python3 doit etre le python systeme 3.12 (rclpy, tkinter)
unset PYENV_VERSION VIRTUAL_ENV
# Snap VS Code : ses GTK_PATH/LOCPATH/GIO... font planter rviz2 (symbol lookup error core20)
unset LOCPATH GTK_PATH GTK_EXE_PREFIX GIO_MODULE_DIR GTK_IM_MODULE_FILE GSETTINGS_SCHEMA_DIR GDK_BACKEND
case "${XDG_DATA_DIRS:-}" in *snap/code*) export XDG_DATA_DIRS=/usr/local/share:/usr/share ;; esac
case "${XDG_DATA_HOME:-}" in *snap/code*) export XDG_DATA_HOME="$HOME/.local/share" ;; esac
PATH="$(printf '%s' "$PATH" | tr ':' '\n' | grep -v -e '/snap/code' -e '\.pyenv' | paste -sd ':')"
export PATH="/usr/bin:/bin:$PATH"

source /opt/ros/jazzy/setup.bash
source "$ROBY_WS/install/setup.bash"

if [ "${ROBY_SIM:-0}" = "1" ]; then
    export ROS_DOMAIN_ID=43
else
    export ROBY_SIM=0
    export ROS_DOMAIN_ID=42
fi
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://$HOME/cyclone_config.xml"
