#!/usr/bin/env python3
import sys, os, tempfile
import mujoco
import mujoco.viewer

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", ".."))
import config
from franka_gripper.gripper.build_gripper import build

XML_PATH = build(lay_flat=True)

# Read the gripper XML and insert a sphere at the centre of the two fingers
SPHERE_RADIUS = 0.030
SPHERE_X = 0.085

with open(XML_PATH) as f:
    xml = f.read()

sphere_block = f'''
    <body name="sphere" pos="{SPHERE_X} 0 {SPHERE_RADIUS}">
      <geom type="sphere" size="{SPHERE_RADIUS}" rgba="0.2 0.6 0.9 1" mass="0.010"
            friction="{config.GRIPPER_OBJECT_FRICTION}"/>
      <freejoint/>
    </body>
'''

xml = xml.replace('</worldbody>', sphere_block + '  </worldbody>')

tmp_path = os.path.join(_HERE, "scene_grasp.xml")
with open(tmp_path, "w") as f:
    f.write(xml)

model = mujoco.MjModel.from_xml_path(tmp_path)
data = mujoco.MjData(model)

Lrest = {}
for f in ("a", "b"):
    tid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, f"{f}_flexor")
    Lrest[f] = float(model.tendon_lengthspring[tid, 0])

target_dL = 0.0

def key_cb(key):
    global target_dL
    step = 10.0
    if key == 265:
        target_dL = min(target_dL + step, config.GRIPPER_MAX_PULL_MM)
        print(f"  target ΔL = {target_dL:.1f} mm")
    elif key == 264:
        target_dL = max(target_dL - step, 0.0)
        print(f"  target ΔL = {target_dL:.1f} mm")
    elif key in (82, 114):
        target_dL = 0.0
        print("  ΔL = 0 (reset)")

with mujoco.viewer.launch_passive(model, data, key_callback=key_cb) as viewer:
    mujoco.mjv_defaultFreeCamera(model, viewer.cam)
    viewer.cam.distance = 0.8
    viewer.cam.azimuth = 90
    viewer.cam.elevation = -20

    print("\nSpheres at centre of fingers — ↑/↓ to pull tendon, R reset")

    dL = 0.0
    while viewer.is_running():
        dL += (target_dL / 1000.0 - dL) * 0.1
        for f in ("a", "b"):
            tid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, f"{f}_flexor")
            ls = Lrest[f] - dL
            model.tendon_lengthspring[tid] = (ls, ls)
        mujoco.mj_step(model, data)
        viewer.sync()
