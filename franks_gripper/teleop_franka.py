import threading
import glfw
import numpy as np
import mujoco
import mujoco.viewer
import time
import sys

_HERE = "/home/sarveshp/underactuated-hand-sim/franka_gripper"
XML_PATH = f"{_HERE}/scene.xml"

JOINT_NAMES = [f"joint{i}" for i in range(1, 8)]
ARM_RANGES = [
    (-2.8973, 2.8973), (-1.7628, 1.7628), (-2.8973, 2.8973),
    (-3.0718, -0.0698), (-2.8973, 2.8973), (-0.0175, 3.7525),
    (-2.8973, 2.8973),
]
HOME_QPOS = np.array([0.0, 0.0, 0.0, -1.57079, 0.0, 1.57079, -0.7853])
HOME_FINGER_CTRL = 255.0
STEP = 0.5

KEY_MAP = {
    glfw.KEY_1: (0, +STEP),  glfw.KEY_2: (0, -STEP),
    glfw.KEY_3: (1, +STEP),  glfw.KEY_4: (1, -STEP),
    glfw.KEY_5: (2, +STEP),  glfw.KEY_6: (2, -STEP),
    glfw.KEY_7: (3, +STEP),  glfw.KEY_8: (3, -STEP),
    glfw.KEY_9: (4, +STEP),  glfw.KEY_0: (4, -STEP),
    glfw.KEY_RIGHT: (5, +STEP), glfw.KEY_LEFT: (5, -STEP),
    glfw.KEY_UP: (6, +STEP),    glfw.KEY_DOWN: (6, -STEP),
    glfw.KEY_LEFT_BRACKET: (7, -255.0),   # [ = open
    glfw.KEY_RIGHT_BRACKET: (7, +255.0),  # ] = close
    glfw.KEY_EQUAL: None,  # = home
}


class FrankaState:
    def __init__(self):
        self.lock = threading.Lock()
        self.qpos_target = HOME_QPOS.copy()
        self.finger_ctrl_target = HOME_FINGER_CTRL
        self.qpos_actual = HOME_QPOS.copy()
        self.finger_aperture = 0.04
        self.finger_ctrl_actual = HOME_FINGER_CTRL
        self.running = True


def sim_thread(state: FrankaState, key_callback):
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)

    qpos_full = np.zeros(model.nq)
    qpos_full[:7] = HOME_QPOS
    qpos_full[7:] = 0.04
    data.qpos[:] = qpos_full
    mujoco.mj_forward(model, data)

    with mujoco.viewer.launch_passive(
        model, data, key_callback=key_callback,
        show_left_ui=True, show_right_ui=True,
    ) as viewer:
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)
        viewer.cam.distance = 2.5
        viewer.cam.azimuth = 120
        viewer.cam.elevation = -25
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = False
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = False
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = False

        while viewer.is_running() and state.running:
            # Override MuJoCo's number-key geom toggles every frame
            viewer.opt.geomgroup[:] = 0
            viewer.opt.geomgroup[0] = 1
            viewer.opt.geomgroup[2] = 1

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

            viewer.clear_texts()
            viewer.sync()
            time.sleep(0.001)

    state.running = False


def main():
    state = FrankaState()

    def key_callback(key):
        with state.lock:
            if key in KEY_MAP:
                entry = KEY_MAP[key]
                if entry is None:
                    state.qpos_target = HOME_QPOS.copy()
                    state.finger_ctrl_target = HOME_FINGER_CTRL
                    print("  → home")
                else:
                    idx, delta = entry
                    if idx < 7:
                        lo, hi = ARM_RANGES[idx]
                        state.qpos_target[idx] = np.clip(
                            state.qpos_target[idx] + delta, lo, hi)
                        print(f"  joint{idx+1} {'+' if delta>0 else '-'}{abs(delta)} → {state.qpos_target[idx]:+.3f}")
                    else:
                        state.finger_ctrl_target = np.clip(delta, 0, 255)
                        print(f"  fingers {'close' if delta>0 else 'open'}")
            sys.stdout.flush()

    t = threading.Thread(
        target=sim_thread, args=(state, key_callback), daemon=True)
    t.start()

    print("=" * 50)
    print(" Franka Panda — Keyboard Teleop")
    print("=" * 50)
    print("  joint1:  1 (+)   2 (-)")
    print("  joint2:  3 (+)   4 (-)")
    print("  joint3:  5 (+)   6 (-)")
    print("  joint4:  7 (+)   8 (-)")
    print("  joint5:  9 (+)   0 (-)")
    print("  joint6:  → (+)   ← (-)")
    print("  joint7:  ↑ (+)   ↓ (-)")
    print("  fingers: [ (open)   ] (close)")
    print("  = — home   Ctrl+C — quit")
    print("=" * 50)
    print("(number keys won't affect visuals — overridden every frame)")
    print("=" * 50)
    sys.stdout.flush()

    try:
        while state.running:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nQuitting...")
        state.running = False


if __name__ == "__main__":
    main()
