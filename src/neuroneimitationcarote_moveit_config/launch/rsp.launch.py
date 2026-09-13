import os
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    # Refus du domaine 42 (2026-09-13, spec-point-entree-unique-lancement) : ce lancement
    # publie l'URDF MOCK du moveit_config ; a cote du vrai bras, c'est la course au mock (steppers morts, BUG-008).
    if os.environ.get("ROS_DOMAIN_ID") == "42":
        raise RuntimeError(
            "rsp.launch.py refuse sur le domaine 42 (vrai robot) : il publie l'URDF MOCK du moveit_config. "
            "Vrai robot : `roby up --nid-confirme` ; simulation : `roby up --sim`."
        )
    moveit_config = MoveItConfigsBuilder("neuroneimitationcarote", package_name="neuroneimitationcarote_moveit_config").to_moveit_configs()
    return generate_rsp_launch(moveit_config)
