"""
Spec de la stack Roby — extraite des launch/scripts réels (2026-07-07).

C'est la *source de vérité* du superviseur : chaque "service" décrit
- où il tourne (host),
- si le superviseur possède son cycle de vie (owned) — sinon il est lancé
  "par décision" et on se contente de SIGNALER son état, on ne le tue pas,
- s'il est critique (critical) — un critique malsain bloque le "prêt",
- comment vérifier sa PRÉSENCE (nodes attendus) et sa SANTÉ (topic vivant),
- comment le (re)lancer (launch) — utilisé plus tard, pas dans le diagnostic.

⚠️ Distinction clé (demande de Sam) : un service peut être PRÉSENT (node listé)
mais MALSAIN (topic qui ne circule plus, zombie). D'où deux checks séparés :
`nodes` (présence) et `live` (santé).

Architecture courante : Pi5 = robot_control.launch.py (RT), PC = pc_moveit.launch.py.
"""

PI5 = "192.168.2.37"
PC = "local"

# Un "live check" décrit comment prouver que le service est VIVANT, pas juste listé :
#   {"topic": "/x", "kind": "flow"}             -> un message doit circuler (echo --once)
#   {"topic": "/x", "kind": "pub"}              -> le topic doit avoir >=1 publisher
#   {"topic": "/x", "kind": "sub"}              -> le topic doit avoir >=1 subscriber

STACK = [
    # ---------- Pi5 : temps réel (robot_control.launch.py) — POSSÉDÉ, CRITIQUE ----------
    {
        "name": "rsp",  # robot_state_publisher : UNIQUE publisher de /robot_description
        "host": PI5, "owned": True, "critical": True,
        "nodes": ["/robot_state_publisher"],
        "live": {"topic": "/robot_description", "kind": "pub"},
        "launch": "ros2 launch roby_hardware robot_control.launch.py",  # (lance tout le groupe Pi5)
        # ⚠️ ce launch ACTIVE arm_controller → risque d'à-coup moteur (BUG-006).
        # Repair refuse de le (re)lancer sans le drapeau --go-motors.
        "moves_motors": True,
        "proc_match": ["robot_control.launch.py", "ros2_control_node",
                       "robot_state_publisher"],
    },
    {
        "name": "ros2_control",  # controller_manager + hardware RobySystem
        "host": PI5, "owned": True, "critical": True,
        "nodes": ["/controller_manager", "/robysystem"],
        "live": {"topic": "/joint_states", "kind": "flow"},  # le pouls du robot
        "launch": None,  # démarré par le même launch que rsp
    },
    {
        "name": "joint_state_broadcaster",
        "host": PI5, "owned": True, "critical": True,
        "nodes": ["/joint_state_broadcaster"],
        "live": {"topic": "/joint_states", "kind": "pub"},
        "launch": None,
    },
    {
        "name": "arm_controller",
        "host": PI5, "owned": True, "critical": True,
        "nodes": ["/arm_controller"],
        "live": None,  # présence suffit ici ; l'activation se vérifie via controller_manager (plus tard)
        "launch": None,
    },

    # ---------- PC : MoveIt (pc_moveit.launch.py) ----------
    {
        "name": "move_group",
        "host": PC, "owned": True, "critical": True,
        "nodes": ["/move_group"],
        "live": {"topic": "/robot_description_semantic", "kind": "pub"},
        # pc_moveit.launch.py amène move_group + rviz ensemble (archi PC documentée).
        "launch": "ros2 launch neuroneimitationcarote_moveit_config pc_moveit.launch.py",
        "proc_match": ["pc_moveit.launch.py", "moveit_ros_move_group/move_group"],
    },
    {
        "name": "rviz",
        "host": PC, "owned": True, "critical": False,  # jetable : visualisation
        "nodes": ["/rviz"],
        "live": None,
        "launch": "ros2 launch neuroneimitationcarote_moveit_config rviz_only.launch.py",
        # signatures process pour tuer les restes avant relance (idempotence)
        "proc_match": ["rviz_only.launch.py", "lib/rviz2/rviz2"],
    },

    # ---------- Lancés PAR DÉCISION (non possédés) : on SIGNALE, on ne tue pas ----------
    {
        "name": "gripper_node",
        "host": PI5, "owned": False, "critical": False,
        "nodes": [],  # nom de node non fiable ; on juge par l'abonnement au topic
        "live": {"topic": "/gripper", "kind": "sub"},  # le node écoute /gripper
        "launch": "python3 ~/gripper_node.py",
    },
    {
        "name": "head_lock_node",
        "host": PI5, "owned": False, "critical": False,
        "nodes": [],
        "live": {"topic": "/head_lock", "kind": "sub"},
        "launch": "python3 ~/head_lock_node.py",
    },
]

# Préconditions infra à vérifier AVANT de juger les services (lecture seule).
INFRA = {
    "pi5_ip": PI5,
    "clock_skew_tol_s": 2.0,          # tolérance de désynchro horloge PC<->Pi5
    "pc_dds_iface": "enp86s0",        # cyclone_config.xml du PC doit pointer ça
    "pc_dds_config": "/home/sam/cyclone_config.xml",
}
