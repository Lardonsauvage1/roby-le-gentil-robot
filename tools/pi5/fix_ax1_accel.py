import time
INST="/home/roby/rlgr/install/roby_hardware/share/roby_hardware/launch/robot_full.launch.py"
SRC="/home/roby/rlgr/src/roby_hardware/launch/robot_full.launch.py"
FAST="/home/roby/move_group_fast.launch.py"

old_loop = 'for joint in ["joint_1", "joint_2", "joint_3"]:'
new_loop = 'for joint in ["joint_2", "joint_3"]:'
rf_anchor = '            "max_acceleration": 6.4,\n        }'
rf_inject = rf_anchor + '\n    limits["joint_1"] = {\n        "has_velocity_limits": True,\n        "max_velocity": 3.2,\n        "has_acceleration_limits": True,\n        "max_acceleration": 3.0,\n    }'
for p in (INST, SRC):
    s = open(p).read()
    open(p + ".bak_ax1_%d" % int(time.time()), "w").write(s)
    assert old_loop in s and rf_anchor in s, "anchor manquant dans " + p
    s = s.replace(old_loop, new_loop, 1).replace(rf_anchor, rf_inject, 1)
    open(p, "w").write(s)
    print("edited", p)

fold = 'for j in ["joint_1", "joint_2", "joint_3"]:'
fnew = 'for j in ["joint_2", "joint_3"]:'
mg_anchor = '"has_acceleration_limits": True, "max_acceleration": 6.4}'
mg_inject = mg_anchor + '\n    lim["joint_1"] = {"has_velocity_limits": True, "max_velocity": 3.2,\n                      "has_acceleration_limits": True, "max_acceleration": 3.0}'
s = open(FAST).read()
open(FAST + ".bak_ax1_%d" % int(time.time()), "w").write(s)
assert fold in s and mg_anchor in s, "anchor manquant dans FAST"
s = s.replace(fold, fnew, 1).replace(mg_anchor, mg_inject, 1)
open(FAST, "w").write(s)
print("edited", FAST)
