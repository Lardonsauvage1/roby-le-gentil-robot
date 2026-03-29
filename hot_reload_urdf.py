#!/usr/bin/env python3
"""
hot_reload_urdf.py
Surveille le URDF et recharge RViz automatiquement à chaque sauvegarde.
Usage : python3 ~/ros2_ws/hot_reload_urdf.py
"""

import subprocess
import time
import os
import threading
import yaml

URDF_PATH = os.path.expanduser(
    "~/ros2_ws/src/neuroneimitationcarote_description/urdf/robot.urdf.xacro"
)
PARAMS_TMP = "/tmp/robot_hot_reload_params.yaml"

RVIZ_CONFIG = os.path.expanduser("~/.rviz2/hot_reload.rviz")
RVIZ_CONFIG_CONTENT = """
Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Collision Enabled: true
      Description Source: Topic
      Description Topic: /robot_description
      Name: RobotModel
      Value: true
      Visual Enabled: true
  Fixed Frame: base_link
"""

def write_rviz_config():
    os.makedirs(os.path.dirname(RVIZ_CONFIG), exist_ok=True)
    with open(RVIZ_CONFIG, "w") as f:
        f.write(RVIZ_CONFIG_CONTENT)

def xacro_compile():
    """Compile le xacro. Retourne le contenu URDF ou None si erreur."""
    result = subprocess.run(["xacro", URDF_PATH], capture_output=True, text=True)
    if result.returncode != 0:
        print(f"\n⚠️  Erreur xacro :\n{result.stderr}")
        return None
    return result.stdout

def write_params_yaml(urdf_content):
    """Écrit le URDF dans un fichier YAML ROS 2. Le YAML dump gère l'échappement."""
    params = {
        "robot_state_publisher": {
            "ros__parameters": {
                "robot_description": urdf_content
            }
        }
    }
    with open(PARAMS_TMP, "w") as f:
        yaml.dump(params, f, default_flow_style=False, allow_unicode=True)

def launch_rsp():
    """Lance robot_state_publisher via fichier YAML."""
    content = xacro_compile()
    if content is None:
        return None
    write_params_yaml(content)
    proc = subprocess.Popen([
        "ros2", "run", "robot_state_publisher", "robot_state_publisher",
        "--ros-args", "--params-file", PARAMS_TMP
    ])
    return proc

def launch_jsp():
    return subprocess.Popen(
        ["ros2", "run", "joint_state_publisher_gui", "joint_state_publisher_gui"]
    )

def launch_rviz():
    write_rviz_config()
    return subprocess.Popen(["ros2", "run", "rviz2", "rviz2", "-d", RVIZ_CONFIG])

# Référence globale pour pouvoir tuer/relancer depuis le thread
rsp_proc = None
rsp_lock = threading.Lock()

def reload_urdf():
    """Recompile le URDF et relance robot_state_publisher."""
    global rsp_proc
    content = xacro_compile()
    if content is None:
        return
    write_params_yaml(content)
    with rsp_lock:
        if rsp_proc and rsp_proc.poll() is None:
            rsp_proc.terminate()
            rsp_proc.wait()
        rsp_proc = subprocess.Popen([
            "ros2", "run", "robot_state_publisher", "robot_state_publisher",
            "--ros-args", "--params-file", PARAMS_TMP
        ])
    print("✓")

def watch_file(path, callback, interval=0.5):
    last_mtime = None
    while True:
        try:
            mtime = os.path.getmtime(path)
            if last_mtime is not None and mtime != last_mtime:
                callback()
            last_mtime = mtime
        except FileNotFoundError:
            pass
        time.sleep(interval)

def main():
    global rsp_proc

    print("━" * 50)
    print("  Hot Reload URDF")
    print("━" * 50)
    print(f"  {URDF_PATH}")
    print("  Sauvegarde dans VSCode → mise à jour auto")
    print("  Ctrl+C pour quitter")
    print("━" * 50)

    print("\n  Démarrage des nœuds ROS...")
    rsp_proc = launch_rsp()
    if rsp_proc is None:
        print("  ⚠️  Impossible de lancer robot_state_publisher")
        return

    time.sleep(2)
    jsp_proc = launch_jsp()
    time.sleep(0.5)
    rviz_proc = launch_rviz()
    print("  ✓ RViz ouvert\n")

    def on_change():
        print("  🔄 Rechargement...", end=" ", flush=True)
        reload_urdf()

    threading.Thread(
        target=watch_file,
        args=(URDF_PATH, on_change),
        daemon=True
    ).start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n  Arrêt...")
        for proc in [rsp_proc, jsp_proc, rviz_proc]:
            if proc and proc.poll() is None:
                proc.terminate()

if __name__ == "__main__":
    main()
