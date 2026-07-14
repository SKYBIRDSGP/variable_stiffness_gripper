#!/usr/bin/env python3

import threading
import math
import time
import tkinter as tk

import numpy as np
import mujoco
import mujoco.viewer

_HERE = "/home/sarveshp/underactuated-hand-sim/franka_gripper/franka_tendon_ee"
XML_PATH = f"{_HERE}/franka_tendon.xml"

JOINT_NAMES = ("mcp", "pip", "dip")
ARM_RANGES = [
    (-2.8973, 2.8973), (-1.7628, 1.7628), (-2.8973, 2.8973),
    (-3.0718, -0.0698), (-2.8973, 2.8973), (-0.0175, 3.7525),
    (-2.8973, 2.8973),
]
DEFAULT_STIFFNESS = 0.81
STIFFNESS_RANGE = (0.0, 3.0)
MAX_DELTA_L_MM = 20.0
SMOOTH = 0.08


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.dL = {"a": 0.0, "b": 0.0}
        self.stiffness = {n: DEFAULT_STIFFNESS for n in JOINT_NAMES}
        self.arm_ctrl = np.array([0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, -0.7853])
        self.reset = False
        self.running = True


def sim_thread(state: State):
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)

    mujoco.mj_forward(model, data)

    # capture initial lengths for tendon ΔL control
    Lrest = {}
    for f in "ab":
        t = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, f"{f}_flexor")
        Lrest[f] = float(model.tendon_lengthspring[t, 0])

    # resolve tendon & joint IDs
    tend_ids = {}
    jnt_ids = {}
    for f in "ab":
        tend_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, f"{f}_flexor")
        tend_ids[f] = tend_id
        jnt_ids[f] = {}
        for n in JOINT_NAMES:
            jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{f}_{n}")
            jnt_ids[f][n] = jnt_id

    cur_dL = {"a": 0.0, "b": 0.0}

    n_sub = max(1, round((1.0 / 120.0) / model.opt.timestep))
    frame_dt = n_sub * model.opt.timestep

    data.qpos[:7] = state.arm_ctrl.copy()
    mujoco.mj_forward(model, data)

    with mujoco.viewer.launch_passive(
        model, data, show_left_ui=True, show_right_ui=True,
    ) as viewer:
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)
        viewer.cam.distance = 2.5
        viewer.cam.azimuth = 120
        viewer.cam.elevation = -25
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = False
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = False
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = False

        while viewer.is_running() and state.running:
            with state.lock:
                snap = type("Snap", (), {})()
                snap.reset = state.reset
                state.reset = False
                snap.dL = state.dL.copy()
                snap.stiffness = state.stiffness.copy()
                snap.arm_ctrl = state.arm_ctrl.copy()

            if snap.reset:
                mujoco.mj_resetData(model, data)
                cur_dL["a"] = cur_dL["b"] = 0.0
                data.qpos[:7] = snap.arm_ctrl

            # joint-stiffness override
            for f in "ab":
                for n in JOINT_NAMES:
                    model.jnt_stiffness[jnt_ids[f][n]] = snap.stiffness[n]

            # tendon ΔL (low-passed, applied as springlength compression)
            for f in "ab":
                cur_dL[f] += (snap.dL[f] - cur_dL[f]) * SMOOTH
                ls = Lrest[f] - cur_dL[f]
                model.tendon_lengthspring[tend_ids[f]] = (ls, ls)

            # arm ctrl
            data.ctrl[:7] = snap.arm_ctrl

            viewer.opt.geomgroup[:] = 0
            viewer.opt.geomgroup[0] = 1
            viewer.opt.geomgroup[2] = 1

            for _ in range(n_sub):
                mujoco.mj_step(model, data)

            with state.lock:
                state.joint_angles = {
                    f: {n: data.qpos[jnt_ids[f][n]] for n in JOINT_NAMES}
                    for f in "ab"
                }
                state.tendon_tension = {}
                for f in "ab":
                    t = tend_ids[f]
                    stretch = max(0.0, float(data.ten_length[t]) - float(model.tendon_lengthspring[t, 0]))
                    state.tendon_tension[f] = float(model.tendon_stiffness[t]) * stretch

            t0 = time.time()
            viewer.sync()
            dt = frame_dt - (time.time() - t0)
            if dt > 0:
                time.sleep(dt)

    state.running = False


def run_dashboard():
    state = State()

    t = threading.Thread(target=sim_thread, args=(state,), daemon=True)
    t.start()

    root = tk.Tk()
    root.title("Franka + Tendon Gripper")
    root.resizable(False, False)

    main = tk.Frame(root, padx=14, pady=14)
    main.pack()

    # ── Arm ──
    arm_vars = []
    arm_labels = []
    tk.Label(main, text="Arm Joints (rad)", font=("Segoe UI", 11, "bold")
             ).grid(row=0, column=0, columnspan=3, pady=(0, 8))
    for i in range(7):
        row = i + 1
        lo, hi = ARM_RANGES[i]
        tk.Label(main, text=f"j{i+1}", width=4, anchor="e"
                 ).grid(row=row, column=0, padx=(0, 6))
        var = tk.DoubleVar(value=state.arm_ctrl[i])
        arm_vars.append(var)
        def _on_arm(idx=i):
            val = arm_vars[idx].get()
            arm_labels[idx].config(text=f"{val:+.3f}")
            with state.lock:
                state.arm_ctrl[idx] = val
        tk.Scale(main, from_=hi, to=lo, resolution=0.02,
                 orient="horizontal", length=220, variable=var,
                 command=lambda _, idx=i: _on_arm(idx)
                 ).grid(row=row, column=1, padx=4)
        lbl = tk.Label(main, text=f"{var.get():+.3f}", width=8, anchor="w",
                       font=("Consolas", 9))
        lbl.grid(row=row, column=2, padx=(4, 0))
        arm_labels.append(lbl)

    row_offset = 9

    # ── ΔL ──
    sep = tk.Frame(main, height=2, bd=1, relief="sunken")
    sep.grid(row=row_offset, column=0, columnspan=3, sticky="ew", pady=6)
    r = row_offset + 1
    tk.Label(main, text="Tendon ΔL (mm)", font=("Segoe UI", 11, "bold")
             ).grid(row=r, column=0, columnspan=3, pady=(0, 6))
    r += 1
    dl_vars = {}
    dl_labels = {}
    for f in ("a", "b"):
        tk.Label(main, text=f"ΔL_{f}", width=4, anchor="e"
                 ).grid(row=r, column=0, padx=(0, 6))
        var = tk.DoubleVar(value=0.0)
        dl_vars[f] = var
        def _on_dl(finger=f):
            val = dl_vars[finger].get()
            dl_labels[finger].config(text=f"{val:5.1f}")
            with state.lock:
                state.dL[finger] = val / 1000.0
        tk.Scale(main, from_=MAX_DELTA_L_MM, to=0.0, resolution=0.1,
                 orient="horizontal", length=220, variable=var,
                 command=lambda _, finger=f: _on_dl(finger)
                 ).grid(row=r, column=1, padx=4)
        lbl = tk.Label(main, text="  0.0", width=6, anchor="w",
                       font=("Consolas", 9))
        lbl.grid(row=r, column=2, padx=(4, 0))
        dl_labels[f] = lbl
        r += 1

    # ── Stiffness ──
    r += 1
    sep2 = tk.Frame(main, height=2, bd=1, relief="sunken")
    sep2.grid(row=r, column=0, columnspan=3, sticky="ew", pady=6)
    r += 1
    tk.Label(main, text="Joint Stiffness (N·m/rad)", font=("Segoe UI", 11, "bold")
             ).grid(row=r, column=0, columnspan=3, pady=(0, 6))
    r += 1
    stiff_vars = {}
    stiff_labels = {}
    for n in JOINT_NAMES:
        tk.Label(main, text=n, width=4, anchor="e"
                 ).grid(row=r, column=0, padx=(0, 6))
        var = tk.DoubleVar(value=DEFAULT_STIFFNESS)
        stiff_vars[n] = var
        def _on_stiff(jname=n):
            val = stiff_vars[jname].get()
            stiff_labels[jname].config(text=f"{val:.3f}")
            with state.lock:
                state.stiffness[jname] = val
        tk.Scale(main, from_=STIFFNESS_RANGE[1], to=STIFFNESS_RANGE[0],
                 resolution=0.01, orient="horizontal", length=220,
                 variable=var,
                 command=lambda _, jname=n: _on_stiff(jname)
                 ).grid(row=r, column=1, padx=4)
        lbl = tk.Label(main, text=f"{var.get():.3f}", width=6, anchor="w",
                       font=("Consolas", 9))
        lbl.grid(row=r, column=2, padx=(4, 0))
        stiff_labels[n] = lbl
        r += 1

    # ── Live readouts ──
    r += 1
    sep3 = tk.Frame(main, height=2, bd=1, relief="sunken")
    sep3.grid(row=r, column=0, columnspan=3, sticky="ew", pady=6)
    r += 1
    tk.Label(main, text="Live Readouts", font=("Segoe UI", 10, "bold")
             ).grid(row=r, column=0, columnspan=3, pady=(0, 6))
    r += 1
    ro_var = tk.StringVar(value="(waiting...)")
    ro_lbl = tk.Label(main, textvariable=ro_var, font=("Consolas", 9),
                      justify="left", anchor="w")
    ro_lbl.grid(row=r, column=0, columnspan=3, padx=10, sticky="w")

    # ── Buttons ──
    btn_row = r + 2
    btn_frame = tk.Frame(main)
    btn_frame.grid(row=btn_row, column=0, columnspan=3, pady=(10, 0))

    def _home():
        for i in range(7):
            arm_vars[i].set(state.arm_ctrl[i])
        for f in "ab":
            dl_vars[f].set(0.0)
        for n in JOINT_NAMES:
            stiff_vars[n].set(DEFAULT_STIFFNESS)
        with state.lock:
            state.arm_ctrl = np.array([0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, -0.7853])
            state.dL = {"a": 0.0, "b": 0.0}
            state.stiffness = {n: DEFAULT_STIFFNESS for n in JOINT_NAMES}
            state.reset = True

    tk.Button(btn_frame, text="Home", command=_home, width=10
              ).pack(side="left", padx=4)
    tk.Button(btn_frame, text="Quit", command=root.destroy, width=10
              ).pack(side="left", padx=4)

    def _poll():
        if not state.running:
            return
        with state.lock:
            ja = getattr(state, "joint_angles", None)
            tt = getattr(state, "tendon_tension", None)
        if ja and tt:
            lines = []
            for f in "ab":
                angles = "  ".join(f"{ja[f][n] * 180 / math.pi:5.1f}°" for n in JOINT_NAMES)
                lines.append(f"Finger {f.upper()}   {angles}   T {tt[f]:6.1f} N")
            ro_var.set("\n".join(lines))
        root.after(50, _poll)

    _poll()

    print("=" * 58)
    print(" Franka + Tendon Gripper — ΔL + Stiffness Dashboard")
    print("=" * 58)
    print("  Arm: joints 1–7 (position sliders)")
    print("  ΔL_a / ΔL_b: tendon pull (0–20 mm)")
    print("  mcp / pip / dip: shared torsional stiffness (N·m/rad)")
    print("  Close window or press Quit to exit.")
    print("=" * 58)

    root.protocol("WM_DELETE_WINDOW", lambda: (
        setattr(state, 'running', False), root.destroy()
    ))
    root.mainloop()
    state.running = False


if __name__ == "__main__":
    run_dashboard()
