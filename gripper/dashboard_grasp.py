#!/usr/bin/env python3
import sys, os, threading, time
import tkinter as tk
from tkinter import ttk

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_HERE, "..", ".."))
import config
from franka_gripper.gripper.build_gripper import build
import mujoco
import mujoco.viewer

XML_PATH = build(lay_flat=True)

with open(XML_PATH) as f:
    xml = f.read()

CUBE_HALF = 0.0175  # 35mm cube

cube_block = f'''
    <body name="cube" pos="0 0 {CUBE_HALF}">
      <joint name="slide_x" type="slide" axis="1 0 0" limited="true" range="-0.05 0.25"
             stiffness="0" damping="0.5"/>
      <geom type="box" size="{CUBE_HALF} {CUBE_HALF} {CUBE_HALF}" rgba="0.2 0.6 0.9 1" mass="0.010"
            friction="{config.GRIPPER_OBJECT_FRICTION}"/>
    </body>
'''

# Soft silicone padding on phalanx contact surfaces
xml = xml.replace(
    '<geom type="mesh" rgba="0.78 0.80 0.85 1" contype="1" conaffinity="1"/>',
    '<geom type="mesh" rgba="0.78 0.80 0.85 1" contype="1" conaffinity="1"'
    ' solref="0.008 0.5" solimp="0.3 0.9 0.01 0.3 2"/>'
)
xml = xml.replace('</worldbody>', cube_block + '  </worldbody>')
scene_path = os.path.join(_HERE, "scene_grasp.xml")
with open(scene_path, "w") as f:
    f.write(xml)

model = mujoco.MjModel.from_xml_path(scene_path)
data = mujoco.MjData(model)

Lrest = {}
for f in ("a", "b"):
    tid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, f"{f}_flexor")
    Lrest[f] = float(model.tendon_lengthspring[tid, 0])

slide_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "slide_x")
slide_qadr = model.jnt_qposadr[slide_id]
slide_dadr = model.jnt_dofadr[slide_id]

data.qpos[slide_qadr] = 0.145
data.qvel[slide_dadr] = 0.0
mujoco.mj_forward(model, data)

dL = 0.0
target_dL = 0.0
target_x = 0.145
lock = threading.Lock()
slider_dirty = False

def key_cb(key):
    global dL, target_dL
    if key == 82:  # R — reset fingers only
        for name in ("a_mcp", "a_pip", "a_dip", "b_mcp", "b_pip", "b_dip"):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            data.qpos[model.jnt_qposadr[jid]] = 0.0
        dL = 0.0
        target_dL = 0.0
        _dl_var_ref.set(0.0)
        dl_label.config(text="0.0")
        with lock:
            data.qpos[slide_qadr] = target_x
            data.qvel[slide_dadr] = 0.0
        mujoco.mj_forward(model, data)

def viewer_loop():
    global dL, target_dL, target_x, slider_dirty
    with mujoco.viewer.launch_passive(model, data, key_callback=key_cb) as viewer:
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)
        viewer.cam.distance = 0.8
        viewer.cam.azimuth = 90
        viewer.cam.elevation = -20

        print("  Drag slider to position sphere  |  R to reset fingers")
        while viewer.is_running():
            with lock:
                tx = target_x
                dirty = slider_dirty
                slider_dirty = False
            dL += (target_dL / 1000.0 - dL) * 0.05
            for f in ("a", "b"):
                tid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, f"{f}_flexor")
                ls = Lrest[f] - dL
                model.tendon_lengthspring[tid] = (ls, ls)
            if dirty:
                data.qpos[slide_qadr] = tx
                data.qvel[slide_dadr] = 0.0
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(1 / 240)

t = threading.Thread(target=viewer_loop, daemon=True)
t.start()

root = tk.Tk()
root.title("Grasp Dashboard")
root.geometry("480x300")

ttk.Label(root, text="Sphere X position (m)").pack(pady=(10, 0))
x_var = tk.DoubleVar(value=0.145)
x_label = ttk.Label(root, text="0.145")
x_label.pack()

def on_x_slider(_=None):
    global target_x, slider_dirty
    with lock:
        target_x = x_var.get()
        slider_dirty = True
    x_label.config(text=f"{target_x:.3f}")
    root.title(f"Grasp — X = {target_x:.3f} m")

x_slider = ttk.Scale(root, from_=0.02, to=0.17, variable=x_var,
                     orient=tk.HORIZONTAL, command=on_x_slider)
x_slider.pack(fill=tk.X, padx=20, pady=5)

ttk.Label(root, text="Tendon pull ΔL (mm)").pack(pady=(10, 0))
dl_var = tk.DoubleVar(value=0)
dl_label = ttk.Label(root, text="0.0")
_dl_var_ref = dl_var  # for key_cb to reset
dl_label.pack()

def on_dl_slider(_=None):
    global target_dL
    with lock:
        target_dL = dl_var.get()
    dl_label.config(text=f"{target_dL:.1f}")

dl_slider = ttk.Scale(root, from_=0, to=config.GRIPPER_MAX_PULL_MM,
                      variable=dl_var, orient=tk.HORIZONTAL, command=on_dl_slider)
dl_slider.pack(fill=tk.X, padx=20, pady=5)

pos_frame = ttk.LabelFrame(root, text="Live sphere position", padding=5)
pos_frame.pack(fill=tk.X, padx=20, pady=10)
pos_label = ttk.Label(pos_frame, text="x = —   y = —   z = —")
pos_label.pack()

def update_pos():
    with lock:
        p = data.body("sphere").xpos
    pos_label.config(text=f"x = {p[0]:.4f}   y = {p[1]:.4f}   z = {p[2]:.4f}")
    root.after(50, update_pos)

root.after(100, update_pos)

def on_close():
    root.destroy()
    os._exit(0)

root.protocol("WM_DELETE_WINDOW", on_close)
root.mainloop()
