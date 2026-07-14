import threading
import numpy as np
import mujoco
import mujoco.viewer
import time

_HERE = "/home/sarveshp/underactuated-hand-sim/franka_gripper"
XML_PATH = f"{_HERE}/scene.xml"

JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
ARM_RANGES = [
    (-2.8973, 2.8973), (-1.7628, 1.7628), (-2.8973, 2.8973),
    (-3.0718, -0.0698), (-2.8973, 2.8973), (-0.0175, 3.7525),
    (-2.8973, 2.8973),
]
HOME_QPOS = np.array([0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, -0.7853])
HOME_FINGER_CTRL = 255.0

STEP = 0.02  # smaller default step for slider smoothness


class FrankaState:
    def __init__(self):
        self.lock = threading.Lock()
        self.qpos_target = HOME_QPOS.copy()
        self.finger_ctrl_target = HOME_FINGER_CTRL
        self.qpos_actual = HOME_QPOS.copy()
        self.finger_aperture = 0.04
        self.finger_ctrl_actual = HOME_FINGER_CTRL
        self.running = True


def sim_thread(state: FrankaState):
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)

    qpos_full = np.zeros(model.nq)
    qpos_full[:7] = HOME_QPOS
    qpos_full[7:] = 0.04
    data.qpos[:] = qpos_full
    mujoco.mj_forward(model, data)

    with mujoco.viewer.launch_passive(model, data, show_left_ui=True, show_right_ui=True) as viewer:
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)
        viewer.cam.distance = 2.5
        viewer.cam.azimuth = 120
        viewer.cam.elevation = -25
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = False
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = False
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = False
        viewer.opt.geomgroup[:] = 0
        viewer.opt.geomgroup[0] = 1  # floor
        viewer.opt.geomgroup[2] = 1  # visual meshes

        while viewer.is_running() and state.running:
            with state.lock:
                target = state.qpos_target.copy()
                fctrl = state.finger_ctrl_target

            data.ctrl[:7] = target
            data.ctrl[7] = fctrl

            mujoco.mj_step(model, data)

            with state.lock:
                state.qpos_actual = data.qpos[:7].copy()
                state.finger_aperture = float(data.qpos[7])
                state.finger_ctrl_actual = float(data.ctrl[7])

            viewer.sync()
            time.sleep(0.001)

    state.running = False


def run_dashboard():
    import tkinter as tk
    from tkinter import ttk

    state = FrankaState()
    t = threading.Thread(target=sim_thread, args=(state,), daemon=True)
    t.start()

    root = tk.Tk()
    root.title("Franka Panda Teleop")
    root.resizable(False, False)

    main = tk.Frame(root, padx=14, pady=14)
    main.pack()

    # ── Arm joint sliders ──
    tk.Label(main, text="Arm Joint Angles (rad)", font=("Segoe UI", 11, "bold")
             ).grid(row=0, column=0, columnspan=3, pady=(0, 8))

    sliders = []
    val_labels = []
    num_vars = []

    for i in range(7):
        row = i + 1
        lo, hi = ARM_RANGES[i]

        tk.Label(main, text=f"joint{i+1}", width=6, anchor="e"
                 ).grid(row=row, column=0, padx=(0, 6))

        var = tk.DoubleVar(value=HOME_QPOS[i])
        num_vars.append(var)
        s = tk.Scale(main, from_=hi, to=lo, resolution=STEP,
                     orient="horizontal", length=250, variable=var,
                     command=lambda _, idx=i: _on_slider(idx))
        s.grid(row=row, column=1, padx=4)
        sliders.append(s)

        lbl = tk.Label(main, text=f"{var.get():+.3f}", width=8, anchor="w",
                       font=("Consolas", 9))
        lbl.grid(row=row, column=2, padx=(4, 0))
        val_labels.append(lbl)

    # ── Finger slider ──
    sep = tk.Frame(main, height=2, bd=1, relief="sunken")
    sep.grid(row=9, column=0, columnspan=3, sticky="ew", pady=8)

    tk.Label(main, text="Fingers", font=("Segoe UI", 10, "bold")
             ).grid(row=10, column=0, columnspan=3, pady=(0, 4))

    finger_var = tk.DoubleVar(value=HOME_FINGER_CTRL)
    tk.Scale(main, from_=255, to=0, resolution=12.75,
             orient="horizontal", length=250, variable=finger_var,
             command=lambda _: _on_finger()
             ).grid(row=11, column=0, columnspan=3, pady=(0, 4))

    finger_lbl = tk.Label(main, text="", font=("Consolas", 9))
    finger_lbl.grid(row=12, column=0, columnspan=3)

    # ── Readouts ──
    sep2 = tk.Frame(main, height=2, bd=1, relief="sunken")
    sep2.grid(row=13, column=0, columnspan=3, sticky="ew", pady=8)

    tk.Label(main, text="Live Readouts", font=("Segoe UI", 10, "bold")
             ).grid(row=14, column=0, columnspan=3, pady=(0, 6))

    readout_labels = []
    for i in range(7):
        lbl = tk.Label(main, text=f"joint{i+1}: —", font=("Consolas", 9), anchor="w")
        lbl.grid(row=15 + i, column=0, columnspan=3, padx=20, sticky="w")
        readout_labels.append(lbl)

    finger_rd = tk.Label(main, text="fingers: —", font=("Consolas", 9), anchor="w")
    finger_rd.grid(row=22, column=0, columnspan=3, padx=20, sticky="w")

    # ── Buttons ──
    btn_frame = tk.Frame(main)
    btn_frame.grid(row=23, column=0, columnspan=3, pady=(10, 0))

    def _home():
        for i in range(7):
            num_vars[i].set(HOME_QPOS[i])
        finger_var.set(HOME_FINGER_CTRL)
        with state.lock:
            state.qpos_target = HOME_QPOS.copy()
            state.finger_ctrl_target = HOME_FINGER_CTRL

    def _on_slider(idx):
        val = num_vars[idx].get()
        val_labels[idx].config(text=f"{val:+.3f}")
        with state.lock:
            state.qpos_target[idx] = val

    def _on_finger():
        val = finger_var.get()
        with state.lock:
            state.finger_ctrl_target = val

    tk.Button(btn_frame, text="Home", command=_home, width=10
              ).pack(side="left", padx=4)
    tk.Button(btn_frame, text="Quit", command=root.destroy, width=10
              ).pack(side="left", padx=4)

    # ── Poll readouts ──
    def _poll():
        if not state.running:
            return
        with state.lock:
            qpos = state.qpos_actual
            apert = state.finger_aperture
            fctrl = state.finger_ctrl_actual
        for i in range(7):
            readout_labels[i].config(
                text=f"joint{i+1}: {qpos[i]:+7.3f} rad")
        pct = apert / 0.04 * 100
        finger_rd.config(
            text=f"fingers: {apert*1000:.1f} mm ({pct:.0f}%)  ctrl={fctrl:.0f}/255")
        root.after(50, _poll)

    _poll()

    print("=" * 50)
    print(" Franka Panda Dashboard")
    print("=" * 50)
    print(" Use the radio buttons + sliders to control joints.")
    print(" Finger slider controls gripper aperture.")
    print(" Live readouts show actual joint positions.")
    print("=" * 50)

    root.protocol("WM_DELETE_WINDOW", lambda: (
        setattr(state, 'running', False), root.destroy()
    ))
    root.mainloop()
    state.running = False


if __name__ == "__main__":
    run_dashboard()
