#!/usr/bin/env python3

import time
import numpy as np
import mujoco
import mujoco.viewer
from scipy.spatial.transform import Rotation as R

_HERE = "/home/sarveshp/underactuated-hand-sim/franka_gripper/franka_tendon_ee"
XML_PATH = f"{_HERE}/pick_place_tendon_scene.xml"

LOCAL_TCP = np.array([0, 0, 0.1204])
HOME_QPOS = np.array([0, 0, 0, -1.57079, 0, 1.57079, -0.7853])
CUBE_INIT = np.array([0.50, 0.0, 0.100])
BUCKET_POS = np.array([0.30, -0.40, 0.0])
GRASP_ROT = R.from_matrix([[-1., 0., 0.], [0., 1., 0.], [0., 0., -1.]])
IK_LAMBDA = 1e-4
SPEED_RAD_PER_S = np.deg2rad(30)

FINGERS = ("a", "b")
JOINT_NAMES = ("mcp", "pip", "dip")

GRASP_DL_MM = 80.0
STIFFNESS_SOFT = 8.0
STIFFNESS_RAMP_MAX = 60.0
STIFFNESS_RAMP_STEPS = 80


def tcp_pose(data, hand_id):
    pos = data.xpos[hand_id].copy()
    mat = data.xmat[hand_id].reshape(3, 3)
    return pos + mat @ LOCAL_TCP, mat


def physics_step(model, data):
    mujoco.mj_step(model, data)


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


def joint_move(data, model, target_qpos, steps=None, viewer=None, speed=1.0,
               hold_finger_qpos=False):
    dt = 2 * model.opt.timestep
    start = data.qpos[:7].copy()
    if steps is None:
        displ = np.max(np.abs(target_qpos - start))
        max_dq_step = SPEED_RAD_PER_S * dt
        steps = max(int(np.ceil(displ / max_dq_step)), 20)

    finger_qpos_start = data.qpos[7:13].copy() if hold_finger_qpos else None

    def iter_fn(i):
        t = (i + 1) / steps
        for j in range(7):
            data.qpos[j] = start[j] + (target_qpos[j] - start[j]) * t
            data.ctrl[j] = data.qpos[j]
        if hold_finger_qpos:
            data.qpos[7:13] = finger_qpos_start
            data.qvel[7:13] = 0.0
        for _ in range(2):
            physics_step(model, data)
        return False

    paced_loop(steps, iter_fn, dt, speed, viewer)


def set_joint_stiffness(model, val):
    for f in FINGERS:
        for n in JOINT_NAMES:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{f}_{n}")
            model.jnt_stiffness[jid] = val


def open_gripper(data, model, steps=200, viewer=None, speed=1.0):
    dt = 2 * model.opt.timestep
    start_ls = {}
    for f in FINGERS:
        tid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, f"{f}_flexor")
        start_ls[f] = model.tendon_lengthspring[tid, 0].copy()

    def iter_fn(i):
        t = (i + 1) / steps
        for f in FINGERS:
            tid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, f"{f}_flexor")
            current = start_ls[f]
            ls = current + t * (0.0 - current)
            model.tendon_lengthspring[tid] = (ls, ls)
        for _ in range(2):
            physics_step(model, data)
        return False

    paced_loop(steps, iter_fn, dt, speed, viewer)


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


SEGMENT_NAMES = ("proximal", "middle", "distal")


def resolve_phalanx_geom_ids(model):
    ids = set()
    for f in FINGERS:
        for s in SEGMENT_NAMES:
            gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{f}_{s}_pad")
            if gid < 0:
                raise RuntimeError(f"geom {f}_{s}_pad not found in model")
            ids.add(gid)
    return ids


TENDON_HISTORY = []


def log_tendon(model, data, label):
    for f in FINGERS:
        tid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, f"{f}_flexor")
        ls = model.tendon_lengthspring[tid, 0]
        act = data.ten_length[tid]
        vel = data.ten_velocity[tid]
        force = model.tendon_stiffness[tid] * (act - ls) + model.tendon_damping[tid] * vel
        TENDON_HISTORY.append((label, f, ls, act, force))


def cube_in_hand_frame(data, hand_id, cube_id):
    hand_pos = data.xpos[hand_id]
    hand_mat = data.xmat[hand_id].reshape(3, 3)
    cube_pos = data.xpos[cube_id]
    return hand_mat.T @ (cube_pos - hand_pos)


def log_contacts(data, phalanx_ids, cube_gid):
    entries = []
    for j in range(data.ncon):
        c = data.contact[j]
        if (c.geom1 in phalanx_ids and c.geom2 == cube_gid) or \
           (c.geom2 in phalanx_ids and c.geom1 == cube_gid):
            entries.append({
                "geom1": c.geom1, "geom2": c.geom2,
                "pos": c.pos.copy(), "frame": c.frame.copy(),
                "dist": c.dist,
            })
    return entries


def pin_cube_at(data, pos):
    data.qpos[13:16] = pos
    data.qpos[16:20] = [1.0, 0.0, 0.0, 0.0]  # identity quaternion
    data.qvel[13:19] = 0.0                     # zero velocity too


def close_gripper_adaptive(data, model, dL_mm, phalanx_ids, cube_gid,
                           viewer=None, speed=1.0, pin_pos=None):
    dt = 2 * model.opt.timestep
    dL = dL_mm / 1000.0

    Lrest = {}
    for f in FINGERS:
        tid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, f"{f}_flexor")
        Lrest[f] = model.tendon_lengthspring[tid, 0].copy()

    set_joint_stiffness(model, STIFFNESS_SOFT)

    close_steps = 500
    ramp_steps = 200
    hold_steps = 500
    total = close_steps + ramp_steps + hold_steps
    contact_detected = False
    ramp_start = -1

    def iter_fn(i):
        nonlocal contact_detected, ramp_start
        # Pin cube for the first close_steps (fingers close around it)
        if pin_pos is not None and i < close_steps:
            pin_cube_at(data, pin_pos)
        for j in range(data.ncon):
            c = data.contact[j]
            g1, g2 = c.geom1, c.geom2
            if (g1 in phalanx_ids and g2 == cube_gid) or \
               (g2 in phalanx_ids and g1 == cube_gid):
                if not contact_detected:
                    contact_detected = True
                    ramp_start = i
                break
        # Phase 1: close softly
        if contact_detected and ramp_start >= 0:
            ramp_elapsed = i - ramp_start
            if ramp_elapsed < ramp_steps:
                frac = ramp_elapsed / ramp_steps
                K = STIFFNESS_SOFT + (STIFFNESS_RAMP_MAX - STIFFNESS_SOFT) * frac
                set_joint_stiffness(model, K)
        # Tendon pull
        t = min((i + 1) / total, 1.0)
        for f in FINGERS:
            tid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, f"{f}_flexor")
            ls = Lrest[f] - t * dL
            model.tendon_lengthspring[tid] = (ls, ls)
        for _ in range(2):
            physics_step(model, data)
        return False

    paced_loop(total, iter_fn, dt, speed, viewer)
    return contact_detected


def grip_lift_ctrl(data, model, hand_id, target_pos, extra_pull_mm,
                   steps=500, viewer=None, speed=1.0, ik_interval=20):
    dt = 2 * model.opt.timestep
    extra_pull = extra_pull_mm / 1000.0
    Lcur = {}
    for f in FINGERS:
        tid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, f"{f}_flexor")
        Lcur[f] = float(model.tendon_lengthspring[tid, 0])
    start_pos, start_mat = tcp_pose(data, hand_id)
    target_rot = R.from_matrix(start_mat)
    current_target = data.qpos[:7].copy()

    def iter_fn(i):
        nonlocal current_target
        t = (i + 1) / steps

        # Rate-limited IK: re-solve every ik_interval steps
        if i % ik_interval == 0:
            desired_pos = start_pos + (target_pos - start_pos) * t
            saved = data.qpos[:7].copy()
            q_solve, _ = solve_ik(data, model, hand_id,
                                  desired_pos, target_rot,
                                  max_steps=200, tol=0.002)
            current_target = q_solve
            data.qpos[:7] = saved
            mujoco.mj_forward(model, data)

        # ctrl-only: let actuator PD track the joint target
        data.ctrl[:7] = current_target

        for f in FINGERS:
            tid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, f"{f}_flexor")
            ls = Lcur[f] - t * extra_pull
            model.tendon_lengthspring[tid] = (ls, ls)

        for _ in range(2):
            physics_step(model, data)
        return False

    paced_loop(steps, iter_fn, dt, speed, viewer)


def joint_move_ctrl(data, model, target_qpos, steps=None, viewer=None, speed=1.0):
    dt = 2 * model.opt.timestep
    start = data.qpos[:7].copy()
    if steps is None:
        displ = np.max(np.abs(target_qpos - start))
        max_dq_step = SPEED_RAD_PER_S * dt
        steps = max(int(np.ceil(displ / max_dq_step)), 20)

    def iter_fn(i):
        t = (i + 1) / steps
        desired = start + (target_qpos - start) * t
        data.ctrl[:7] = desired
        for _ in range(2):
            physics_step(model, data)
        return False

    paced_loop(steps, iter_fn, dt, speed, viewer)


def run():
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)
    hand_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hand")
    cube_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")

    TENDON_HISTORY.clear()

    dt_iter = 2 * model.opt.timestep

    for j in range(7):
        data.qpos[j] = HOME_QPOS[j]
        data.ctrl[j] = HOME_QPOS[j]
    data.qpos[13:16] = CUBE_INIT
    data.qpos[16:20] = [1.0, 0.0, 0.0, 0.0]
    data.qvel[13:19] = 0.0
    mujoco.mj_forward(model, data)

    # Initial warmup with pinned cube
    for _ in range(20):
        pin_cube_at(data, CUBE_INIT)
        physics_step(model, data)
    mujoco.mj_forward(model, data)

    pg_pos = CUBE_INIT + np.array([0, 0, 0.35])
    desc_tgt = np.array([0.50, 0.0, 0.085])
    lift_tgt = np.array([0.50, 0.0, 0.150])
    xfer_tgt = BUCKET_POS + [0, 0, 0.35]
    place_tgt = BUCKET_POS + [0, 0, 0.18]

    pg_qpos, _ = solve_ik(data, model, hand_id, pg_pos, GRASP_ROT)
    desc_qpos, _ = solve_ik(data, model, hand_id, desc_tgt, GRASP_ROT)
    lift_qpos, _ = solve_ik(data, model, hand_id, lift_tgt, GRASP_ROT)
    xfer_qpos, _ = solve_ik(data, model, hand_id, xfer_tgt, GRASP_ROT)
    place_qpos, _ = solve_ik(data, model, hand_id, place_tgt, GRASP_ROT)

    for j in range(7):
        data.qpos[j] = HOME_QPOS[j]
        data.ctrl[j] = HOME_QPOS[j]
    data.qpos[13:16] = CUBE_INIT
    data.qpos[16:20] = [1.0, 0.0, 0.0, 0.0]
    data.qvel[13:19] = 0.0
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

        set_joint_stiffness(model, STIFFNESS_SOFT)

        phalanx_ids = resolve_phalanx_geom_ids(model)
        cube_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "cube_geom")

        phases = [
            ("PRE_GRASP", ("qpos", pg_qpos, None)),
            ("ZERO_GRAV", ("gravity", (0.0, 0.0, 0.0))),
            ("DESCEND",   ("qpos", desc_qpos, None)),
            ("GRIP",      ("grip_adaptive", GRASP_DL_MM)),
            ("LIFT",      ("grip_lift_ctrl", lift_tgt, 0.0, 3000, 20)),
            ("RESTORE_GRAV", ("gravity", (0.0, 0.0, -9.81))),
            ("TRANSFER",  ("joint_ctrl", xfer_qpos, None)),
            ("PLACE",     ("joint_ctrl", place_qpos, None)),
            ("RELEASE",   ("open", None)),
            ("SOFTEN",    ("stiffness", STIFFNESS_SOFT)),
            ("RETRACT",   ("qpos", xfer_qpos, None)),
            ("HOME",      ("qpos", HOME_QPOS, None)),
        ]

        for name, action in phases:
            if not viewer.is_running():
                break

            log_tendon(model, data, f"START_{name}")
            print(f"[{name}] ", end="", flush=True)
            action_type = action[0]

            if action_type == "grip_adaptive":
                dL_mm = action[1]
                print(f"adaptive grip {dL_mm} mm")
                # Pin cube during initial close so fingers close around it
                contact_detected = close_gripper_adaptive(
                    data, model, dL_mm, phalanx_ids, cube_gid,
                    viewer=viewer, pin_pos=CUBE_INIT
                )
                print(f"  contact detected: {contact_detected}")

            elif action_type == "open":
                print("tendon release")
                open_gripper(data, model, viewer=viewer)

            elif action_type == "qpos":
                _, qpos, steps = action[:3]
                hold_fingers = action[3] if len(action) > 3 else False
                label = str(steps) if steps is not None else "auto"
                tag = " (hold fingers)" if hold_fingers else ""
                print(f"qpos teleport {label} steps{tag}")
                if name in ("PRE_GRASP", "DESCEND"):
                    dt = 2 * model.opt.timestep
                    start = data.qpos[:7].copy()
                    if steps is None:
                        displ = np.max(np.abs(qpos - start))
                        max_dq_step = SPEED_RAD_PER_S * dt
                        steps = max(int(np.ceil(displ / max_dq_step)), 20)
                    def iter_fn(i):
                        t = (i + 1) / steps
                        for j in range(7):
                            data.qpos[j] = start[j] + (qpos[j] - start[j]) * t
                            data.ctrl[j] = data.qpos[j]
                        pin_cube_at(data, CUBE_INIT)
                        for _ in range(2):
                            physics_step(model, data)
                        return False
                    paced_loop(steps, iter_fn, dt, 1.0, viewer)
                else:
                    joint_move(data, model, qpos, steps, viewer=viewer,
                               hold_finger_qpos=hold_fingers)

            elif action_type == "joint_ctrl":
                _, qpos, steps = action[:3]
                label = str(steps) if steps is not None else "auto"
                print(f"joint ctrl {label} steps")
                joint_move_ctrl(data, model, qpos, steps, viewer=viewer)

            elif action_type == "stiffness":
                val = action[1]
                print(f"stiffness {val:.1f} N·m/rad")
                set_joint_stiffness(model, val)

            elif action_type == "grip_lift_ctrl":
                _, target_pos, extra_pull, steps, ik_int = action
                print(f"cartesian lift + {extra_pull:.0f} mm extra pull, {steps} steps, ik_interval={ik_int}")
                grip_lift_ctrl(data, model, hand_id, target_pos, extra_pull, steps,
                               viewer=viewer, ik_interval=ik_int)

            elif action_type == "gravity":
                val = action[1]
                print(f"gravity={val}")
                model.opt.gravity[:] = val

            elif action_type == "pull":
                extra_mm = action[1]
                steps = action[2] if len(action) > 2 else 400
                print(f"tighten {extra_mm:.0f} mm, {steps} steps")
                extra = extra_mm / 1000.0
                Lcur = {}
                for f in FINGERS:
                    tid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, f"{f}_flexor")
                    Lcur[f] = float(model.tendon_lengthspring[tid, 0])
                def iter_fn(i):
                    t = (i + 1) / steps
                    for f in FINGERS:
                        tid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_TENDON, f"{f}_flexor")
                        model.tendon_lengthspring[tid] = (Lcur[f] - t * extra, Lcur[f] - t * extra)
                    for _ in range(2):
                        physics_step(model, data)
                    return False
                dt = 2 * model.opt.timestep
                paced_loop(steps, iter_fn, dt, 1.0, viewer)

        
            log_tendon(model, data, f"END_{name}")
            if name == "DESCEND":
                pos, _ = tcp_pose(data, hand_id)
                actual = data.xpos[cube_id]
                displ = np.linalg.norm(actual - CUBE_INIT)
                print(f"  TCP={pos}, cube displ={displ:.4f}")

            elif name == "GRIP":
                for _ in range(80):
                    t0 = time.time()
                    physics_step(model, data)
                    viewer.sync()
                    precise_wait(t0, dt_iter)
                actual = data.xpos[cube_id]
                print(f"  cube z={actual[2]:.4f}")
                rel = cube_in_hand_frame(data, hand_id, cube_id)
                print(f"  cube in hand frame: x={rel[0]:.4f} y={rel[1]:.4f} z={rel[2]:.4f}")
                for f in FINGERS:
                    for n in JOINT_NAMES:
                        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{f}_{n}")
                        q = data.qpos[model.jnt_qposadr[jid]]
                        print(f"    {f}_{n}={q:.3f} rad")
                contacts = log_contacts(data, phalanx_ids, cube_gid)
                print(f"    finger-cube contacts: {len(contacts)}")
                for ci, c in enumerate(contacts):
                    print(f"      contact {ci}: geom1={c['geom1']} geom2={c['geom2']} "
                          f"dist={c['dist']:.4f} pos=({c['pos'][0]:.3f},{c['pos'][1]:.3f},{c['pos'][2]:.3f})")

            elif name in ("TIGHTEN", "PULL_LIFT", "TRANSFER", "PLACE", "RELEASE", "LIFT"):
                actual = data.xpos[cube_id]
                print(f"  cube z={actual[2]:.4f}")
                rel = cube_in_hand_frame(data, hand_id, cube_id)
                print(f"  cube in hand frame: x={rel[0]:.4f} y={rel[1]:.4f} z={rel[2]:.4f}")
                if name in ("LIFT",):
                    for f in FINGERS:
                        for n in JOINT_NAMES:
                            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{f}_{n}")
                            q = data.qpos[model.jnt_qposadr[jid]]
                            print(f"    {f}_{n}={q:.3f} rad")
                    contacts = log_contacts(data, phalanx_ids, cube_gid)
                    print(f"    finger-cube contacts: {len(contacts)}")
                    for ci, c in enumerate(contacts):
                        print(f"      contact {ci}: geom1={c['geom1']} geom2={c['geom2']} "
                              f"dist={c['dist']:.4f}")
                if name in ("PLACE",):
                    for _ in range(150):
                        t0 = time.time()
                        physics_step(model, data)
                        viewer.sync()
                        precise_wait(t0, dt_iter)
                    actual = data.xpos[cube_id]
                    print(f"  cube settled at z={actual[2]:.4f}")

            elif name == "SOFTEN":
                print(f"  finger joints softened to {STIFFNESS_SOFT} N·m/rad")

            elif name == "HOME":
                print("  done")

        print("\n=== Tendon History ===")
        for lbl, f, ls, act, force in TENDON_HISTORY:
            print(f"  {lbl:>20s}  {f}  springlen={ls:.6f}  actuallen={act:.6f}  "
                  f"diff={(act-ls)*1000:.2f}mm  force={force:.3f}N")
        print("\n=== Complete! Viewer stays open ===")
        while viewer.is_running():
            physics_step(model, data)
            viewer.sync()


if __name__ == "__main__":
    run()
