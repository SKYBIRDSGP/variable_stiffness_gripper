#!/usr/bin/env python3

import time
import numpy as np
import mujoco
import mujoco.viewer
from scipy.spatial.transform import Rotation as R

_HERE = "/home/sarveshp/underactuated-hand-sim/franka_gripper"
XML_PATH = f"{_HERE}/pick_place_scene.xml"

LOCAL_TCP = np.array([0, 0, 0.0584])
HOME_QPOS = np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853])
SPHERE_INIT = np.array([0.50, 0.0, 0.100])
BUCKET_POS = np.array([0.30, -0.40, 0.0])
GRASP_ROT = R.from_matrix([[-1., 0., 0.], [0., 1., 0.], [0., 0., -1.]])
IK_LAMBDA = 1e-4
SPEED_RAD_PER_S = np.deg2rad(30)
SPHERE_POS_SLICE = slice(9, 12)
SPHERE_QUAT_SLICE = slice(12, 16)
SPHERE_VEL_SLICE = slice(9, 15)
IDENT_Q = np.array([1., 0., 0., 0.])


class SphereCtrl:
    def __init__(self):
        self.override_fn = None

    def set_pin(self):
        self.override_fn = lambda d: self._pin(d)

    def set_attach(self, offset_local, hand_id):
        def _attach(d):
            R = d.xmat[hand_id].reshape(3, 3)
            d.qpos[SPHERE_POS_SLICE] = d.xpos[hand_id] + R @ offset_local
            d.qpos[SPHERE_QUAT_SLICE] = IDENT_Q
            d.qvel[SPHERE_VEL_SLICE] = 0.0
        self.override_fn = _attach

    def clear(self):
        self.override_fn = None

    def apply(self, data):
        if self.override_fn is not None:
            self.override_fn(data)

    @staticmethod
    def _pin(data):
        data.qpos[SPHERE_POS_SLICE] = SPHERE_INIT
        data.qpos[SPHERE_QUAT_SLICE] = IDENT_Q
        data.qvel[SPHERE_VEL_SLICE] = 0.0


_sphere_ctrl = SphereCtrl()


def tcp_pose(data, hand_id):
    pos = data.xpos[hand_id].copy()
    mat = data.xmat[hand_id].reshape(3, 3)
    return pos + mat @ LOCAL_TCP, mat


def physics_step(model, data):
    mujoco.mj_step(model, data)
    _sphere_ctrl.apply(data)


def precise_wait(t0, target_dt):
    elapsed = time.time() - t0
    remaining = target_dt - elapsed
    if remaining <= 0:
        return
    if remaining > 0.003:
        time.sleep(remaining - 0.002)
    while time.time() - t0 < target_dt:
        pass


def paced_loop(steps, iter_fn, dt_sim, speed, viewer):
    if viewer is None:
        for i in range(steps):
            if iter_fn(i):
                break
        return

    target_dt = dt_sim / speed
    for i in range(steps):
        t0 = time.time()
        done = iter_fn(i)
        if viewer is not None:
            viewer.sync()
        precise_wait(t0, target_dt)
        if done:
            break


def joint_move(data, model, target_qpos, steps=None, viewer=None, speed=1.0):
    dt = 2 * model.opt.timestep
    start = data.qpos[:7].copy()
    if steps is None:
        displ = np.max(np.abs(target_qpos - start))
        max_dq_step = SPEED_RAD_PER_S * dt
        steps = max(int(np.ceil(displ / max_dq_step)), 20)

    def iter_fn(i):
        t = (i + 1) / steps
        for j in range(7):
            data.qpos[j] = start[j] + (target_qpos[j] - start[j]) * t
            data.ctrl[j] = data.qpos[j]
        for _ in range(2):
            physics_step(model, data)
        return False

    paced_loop(steps, iter_fn, dt, speed, viewer)


def close_gripper(data, model, close_target=0.0, steps=200, viewer=None, speed=1.0):
    dt = 2 * model.opt.timestep
    data.ctrl[7] = 0

    def iter_fn(i):
        t = (i + 1) / steps
        q = 0.04 - t * (0.04 - close_target)
        data.ctrl[8] = q
        data.ctrl[9] = q
        for _ in range(2):
            physics_step(model, data)
        return False

    paced_loop(steps, iter_fn, dt, speed, viewer)


def open_gripper(data, model, steps=150, viewer=None, speed=1.0):
    dt = 2 * model.opt.timestep

    def iter_fn(i):
        t = (i + 1) / steps
        q = 0.0 + t * 0.04
        data.ctrl[8] = q
        data.ctrl[9] = q
        for _ in range(2):
            physics_step(model, data)
        return False

    paced_loop(steps, iter_fn, dt, speed, viewer)
    data.ctrl[7] = 255


def solve_ik(data, model, hand_id, target_pos, target_rot, max_steps=5000, tol=0.001):
    for i in range(max_steps):
        mujoco.mj_forward(model, data)
        pos, mat = tcp_pose(data, hand_id)
        err_vec = target_pos - pos
        if np.linalg.norm(err_vec) < tol and target_rot is None:
            break
        if target_rot is not None:
            cur = R.from_matrix(mat)
            ori_err = (target_rot * cur.inv()).as_rotvec()
            err = np.concatenate([err_vec, ori_err])
            ndim = 6
            if np.linalg.norm(err_vec) < tol and np.linalg.norm(ori_err) < np.deg2rad(0.5):
                break
        else:
            err = err_vec
            ndim = 3
            if np.linalg.norm(err_vec) < tol:
                break
        jp = np.zeros((3, model.nv))
        jr = np.zeros((3, model.nv))
        mujoco.mj_jac(model, data, jp, jr, pos, hand_id)
        jac = (np.vstack([jp, jr]) if target_rot is not None else jp)[:, :7]
        JJT = jac @ jac.T
        damp = IK_LAMBDA * np.trace(JJT) / ndim + 1e-10
        dq = jac.T @ np.linalg.solve(JJT + damp * np.eye(ndim), err)
        for j in range(7):
            data.qpos[j] += dq[j]
    mujoco.mj_forward(model, data)
    pos, mat = tcp_pose(data, hand_id)
    return data.qpos[:7].copy(), np.linalg.norm(target_pos - pos)


def cartesian_move(data, model, hand_id, target_pos, steps, viewer=None,
                   target_rot=None, speed=1.0, label=""):
    dt = 2 * model.opt.timestep
    max_dq = SPEED_RAD_PER_S * dt

    def iter_fn(i):
        pos, mat = tcp_pose(data, hand_id)
        err_vec = target_pos - pos
        if i > 10 and np.linalg.norm(err_vec) < 0.004:
            return True
        if target_rot is not None:
            cur = R.from_matrix(mat)
            ori_err = (target_rot * cur.inv()).as_rotvec()
            err = np.concatenate([err_vec, ori_err])
            ndim = 6
        else:
            err = err_vec
            ndim = 3
        jp = np.zeros((3, model.nv))
        jr = np.zeros((3, model.nv))
        mujoco.mj_jac(model, data, jp, jr, pos, hand_id)
        jac = (np.vstack([jp, jr]) if target_rot is not None else jp)[:, :7]
        JJT = jac @ jac.T
        damp = IK_LAMBDA * np.trace(JJT) / ndim + 1e-10
        dq = jac.T @ np.linalg.solve(JJT + damp * np.eye(ndim), err)
        nrm = np.linalg.norm(dq)
        if nrm > max_dq:
            dq *= max_dq / nrm
        for j in range(7):
            data.qpos[j] += dq[j]
            data.ctrl[j] = data.qpos[j]
        for _ in range(2):
            physics_step(model, data)
        return False

    paced_loop(steps, iter_fn, dt, speed, viewer)


def run():
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)
    hand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hand")
    sphere_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "sphere")

    dt_iter = 2 * model.opt.timestep

    for j in range(7):
        data.qpos[j] = HOME_QPOS[j]
        data.ctrl[j] = HOME_QPOS[j]
    data.qpos[7] = 0.04
    data.qpos[8] = 0.04
    data.ctrl[7] = 255
    data.ctrl[8] = 0.04
    data.ctrl[9] = 0.04
    mujoco.mj_forward(model, data)

    pg_pos = SPHERE_INIT + np.array([0, 0, 0.35])
    desc_tgt = SPHERE_INIT + [0, 0, 0.043]
    lift_tgt = SPHERE_INIT + [0, 0, 0.35]
    xfer_tgt = BUCKET_POS + [0, 0, 0.35]
    place_tgt = BUCKET_POS + [0, 0, 0.18]

    pg_qpos, _ = solve_ik(data, model, hand_id, pg_pos, GRASP_ROT)
    desc_qpos, _ = solve_ik(data, model, hand_id, desc_tgt, GRASP_ROT)
    lift_qpos, _ = solve_ik(data, model, hand_id, lift_tgt, GRASP_ROT)
    xfer_qpos, _ = solve_ik(data, model, hand_id, xfer_tgt, GRASP_ROT)
    place_qpos, _ = solve_ik(data, model, hand_id, place_tgt, GRASP_ROT)

    # final reset to home before viewer opens
    for j in range(7):
        data.qpos[j] = HOME_QPOS[j]
        data.ctrl[j] = HOME_QPOS[j]
        data.qpos[7] = 0.04
        data.qpos[8] = 0.04
        data.ctrl[7] = 255
        data.ctrl[8] = 0.04
        data.ctrl[9] = 0.04
        mujoco.mj_forward(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        import glfw
        glfw.swap_interval(0)

        mujoco.mjv_defaultFreeCamera(model, viewer.cam)
        viewer.cam.distance = 2.2
        viewer.cam.azimuth = 140
        viewer.cam.elevation = -30
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_TRANSPARENT] = False
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = False
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_JOINT] = False

        phases = [
            ("PRE_GRASP", "pinned",  ("joint", pg_qpos, None)),
            ("DESCEND",   "pinned",  ("joint", desc_qpos, None)),
            ("GRIP",      "grip",    ("close", 0.0)),
            ("LIFT",      "attach",  ("joint", lift_qpos, None)),
            ("TRANSFER",  "attach",  ("joint", xfer_qpos, None)),
            ("PLACE",     "attach",  ("joint", place_qpos, None)),
            ("RELEASE",   "release", ("open", None)),
            ("RETRACT",   "free",    ("joint", xfer_qpos, None)),
            ("HOME",      "free",    ("joint", HOME_QPOS, None)),
        ]

        for name, sphere_mode, action in phases:
            if not viewer.is_running():
                break

            if sphere_mode == "pinned":
                _sphere_ctrl.set_pin()
            elif sphere_mode in ("free", "none"):
                _sphere_ctrl.clear()

            print(f"[{name}] ", end="", flush=True)
            action_type = action[0]

            if action_type == "close":
                print(f"close target={action[1]}")
                close_gripper(data, model, action[1], viewer=viewer)

            elif action_type == "open":
                print("open gripper")
                open_gripper(data, model, viewer=viewer)

            elif action_type == "joint":
                _, qpos, steps = action
                label = str(steps) if steps is not None else "auto"
                print(f"joint move {label} steps")
                joint_move(data, model, qpos, steps, viewer=viewer)

            elif action_type == "cartesian":
                _, pos, steps, orient = action
                rot = GRASP_ROT if orient else None
                print(f"cartesian {steps} steps to {pos} orient={orient}")
                cartesian_move(data, model, hand_id, pos, steps,
                               viewer=viewer, target_rot=rot, label=name)

            if name == "DESCEND":
                pos, _ = tcp_pose(data, hand_id)
                actual = data.xpos[sphere_id]
                displ = np.linalg.norm(actual - SPHERE_INIT)
                print(f"  TCP={pos}, sphere displ={displ:.4f}")

            elif name == "GRIP":
                R = data.xmat[hand_id].reshape(3, 3)
                offset_local = R.T @ (data.xpos[sphere_id] - data.xpos[hand_id])
                print(f"  offset_local = {offset_local}")
                _sphere_ctrl.set_attach(offset_local, hand_id)
                for _ in range(30):
                    t0 = time.time()
                    physics_step(model, data)
                    viewer.sync()
                    precise_wait(t0, dt_iter)

            elif name in ("LIFT", "TRANSFER"):
                actual = data.xpos[sphere_id]
                print(f"  sphere z={actual[2]:.4f}")

            elif name == "PLACE":
                actual = data.xpos[sphere_id]
                print(f"  sphere z={actual[2]:.4f}")
                _sphere_ctrl.clear()
                for _ in range(150):
                    t0 = time.time()
                    physics_step(model, data)
                    viewer.sync()
                    precise_wait(t0, dt_iter)
                actual = data.xpos[sphere_id]
                print(f"  sphere settled at z={actual[2]:.4f}")

            elif name == "RELEASE":
                actual = data.xpos[sphere_id]
                print(f"  sphere z={actual[2]:.4f}")

            elif name == "HOME":
                print("  done")

        print("\n=== Complete! Viewer stays open ===")
        while viewer.is_running():
            physics_step(model, data)
            viewer.sync()


if __name__ == "__main__":
    run()
