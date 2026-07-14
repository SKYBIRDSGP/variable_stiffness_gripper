#!/usr/bin/env python3
import os, sys, json, math
import numpy as np
from scipy.spatial.transform import Rotation as Rot

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.abspath(os.path.join(os.path.expanduser("~"), "underactuated-hand-sim"))
sys.path.insert(0, _HERE)
import config

MM = 1e-3
XML_PATH = os.path.join(_HERE, "gripper.xml")
_PARAMS = os.path.join(_REPO, "high_fidelity", "params.json")
_MESHDIR = os.path.relpath(os.path.join(_REPO, "high_fidelity", "meshes"), _HERE)
JOINT_NAMES = ("mcp", "pip", "dip")


def _load_params():
    with open(_PARAMS) as f:
        return json.load(f)


def _v3(a):
    return f"{a[0]:.6f} {a[1]:.6f} {a[2]:.6f}"


def _q4(q):
    return f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}"


def _rng(r):
    return f"{r[0] * math.pi / 180:.5f} {r[1] * math.pi / 180:.5f}"


def _fi(P, nm):
    I = np.array(P["inertial"][nm]["I"])
    return f"{I[0,0]:.6e} {I[1,1]:.6e} {I[2,2]:.6e} {I[0,1]:.6e} {I[0,2]:.6e} {I[1,2]:.6e}"


def _com(P, nm):
    c = P["inertial"][nm]["com_body"]
    return f"{c[0]:.6f} {c[1]:.6f} {c[2]:.6f}"


_MASS_MAP = {"proximal": config.PROXIMAL_MASS,
             "middle": config.MIDDLE_MASS,
             "distal": config.DISTAL_MASS}


def _mass(P, nm):
    return _MASS_MAP[nm]


def _finger_body_xml(prefix, base_pos, base_quat, P):
    mcp = np.array(config.MCP_CENTER)
    pip = np.array(config.PIP_CENTER)
    dip = np.array(config.DIP_CENTER)
    tip = np.array(config.TIP_POINT)

    pos_prox = mcp * MM
    pos_mid = (pip - mcp) * MM
    pos_dist = (dip - pip) * MM
    tip_local = (tip - dip) * MM

    st = (config.MCP_STIFFNESS, config.PIP_STIFFNESS, config.DIP_STIFFNESS)
    dp = (config.MCP_DAMPING, config.PIP_DAMPING, config.DIP_DAMPING)
    rg = (config.MCP_RANGE, config.PIP_RANGE, config.DIP_RANGE)
    j = [f"{prefix}_{n}" for n in JOINT_NAMES]

    bl_hx = config.BASE_LINK_LENGTH_MM / 2 * MM
    bl_hy = config.BASE_LINK_WIDTH_MM / 2 * MM
    bl_hz = config.BASE_LINK_HEIGHT_MM / 2 * MM
    bl_cx = (config.MCP_CENTER[0] + config.BASE_LINK_INNER_GAP_MM
             + config.BASE_LINK_LENGTH_MM / 2) * MM

    return f'''
    <body name="{prefix}_base" pos="{_v3(base_pos)}" quat="{_q4(base_quat)}">
      <geom type="box" size="{bl_hx:.6f} {bl_hy:.6f} {bl_hz:.6f}" pos="{bl_cx:.6f} 0 0"
            rgba="0.30 0.30 0.33 1" contype="0" conaffinity="0" density="0" mass="0"/>

      <body name="{prefix}_proximal" pos="{_v3(pos_prox)}">
        <joint name="{j[0]}" stiffness="{st[0]}" damping="{dp[0]}" range="{_rng(rg[0])}"/>
        <inertial pos="{_com(P, 'proximal')}" mass="{_mass(P, 'proximal'):.6f}" fullinertia="{_fi(P, 'proximal')}"/>
        <geom class="phalanx" mesh="proximal_mesh"/>

        <body name="{prefix}_middle" pos="{_v3(pos_mid)}">
          <joint name="{j[1]}" stiffness="{st[1]}" damping="{dp[1]}" range="{_rng(rg[1])}"/>
          <inertial pos="{_com(P, 'middle')}" mass="{_mass(P, 'middle'):.6f}" fullinertia="{_fi(P, 'middle')}"/>
          <geom class="phalanx" mesh="middle_mesh"/>

          <body name="{prefix}_distal" pos="{_v3(pos_dist)}">
            <joint name="{j[2]}" stiffness="{st[2]}" damping="{dp[2]}" range="{_rng(rg[2])}"/>
            <inertial pos="{_com(P, 'distal')}" mass="{_mass(P, 'distal'):.6f}" fullinertia="{_fi(P, 'distal')}"/>
            <geom class="phalanx" mesh="distal_mesh"/>
            <site name="{prefix}_tip" pos="{_v3(tip_local)}" size="0.001" rgba="0.2 0.7 0.95 1"/>
          </body>
        </body>
      </body>
    </body>'''


def build(separation=config.GRIPPER_SEPARATION, mount_height=config.GRIPPER_MOUNT_HEIGHT,
          lay_flat=False):
    P = _load_params()
    arm = config.SHEATH_MOMENT_ARM

    R_A = Rot.from_euler("y", 90, degrees=True)
    R_B = Rot.from_euler("z", 180, degrees=True) * R_A
    x, y, z, w = R_A.as_quat()
    qA = (w, x, y, z)
    x, y, z, w = R_B.as_quat()
    qB = (w, x, y, z)

    if lay_flat:
        R_flat = Rot.from_euler("y", 90, degrees=True)
        xf, yf, zf, wf = R_flat.as_quat()
        flat_attrib = f' quat="{wf:.6f} {xf:.6f} {yf:.6f} {zf:.6f}"'
        flat_z = 0.015  # raise so thickest mesh (distal half_x=14.2mm) clears ground
    else:
        flat_attrib = ""
        flat_z = 0.0

    posA = (0.0, +separation / 2.0, mount_height)
    posB = (0.0, -separation / 2.0, mount_height)

    finger_a = _finger_body_xml("a", posA, qA, P)
    finger_b = _finger_body_xml("b", posB, qB, P)

    tendons = "\n".join(
        f'    <fixed name="{f}_flexor" stiffness="{config.TENDON_STIFFNESS}" '
        f'damping="{config.TENDON_DAMPING}" springlength="-1">\n'
        + "\n".join(f'      <joint joint="{f}_{n}" coef="{-arm}"/>' for n in JOINT_NAMES)
        + "\n    </fixed>"
        for f in ("a", "b")
    )

    excludes = "\n".join(
        f'    <exclude body1="{p}_{a}" body2="{p}_{b}"/>'
        for p in ("a", "b")
        for a, b in (("proximal", "middle"), ("middle", "distal"), ("proximal", "distal"))
    )

    bhx = config.GRIPPER_BLOCK_SIZE_MM[0] / 2 * MM
    bhy = config.GRIPPER_BLOCK_SIZE_MM[1] / 2 * MM
    bhz = config.GRIPPER_BLOCK_SIZE_MM[2] / 2 * MM
    block_back_m = (config.MCP_CENTER[0] + config.BASE_LINK_LENGTH_MM) / 1000.0
    block_z = mount_height - block_back_m - bhz

    xml = f'''<mujoco model="tendon_gripper">
  <compiler angle="radian" meshdir="{_MESHDIR}" autolimits="true"/>
  <option timestep="{config.GRIPPER_TIMESTEP}" integrator="{config.INTEGRATOR}"
          gravity="0 0 -9.81" cone="{config.GRIPPER_FRICTION_CONE}" impratio="{config.GRIPPER_IMPRATIO}"/>
  <default>
    <geom condim="6" friction="2.0 2.0 0.05"
          solref="0.0005 1" solimp="0.99 0.9999 0.0001 0.5 2" density="0"/>
    <joint type="hinge" axis="0 0 1" damping="0.08" limited="true"
           solreflimit="0.002 1" solimplimit="0.99 0.9999 0.0001 0.5 2"/>
    <default class="phalanx">
      <geom type="mesh" rgba="0.78 0.80 0.85 1" contype="1" conaffinity="1"/>
    </default>
    <site group="3"/>
  </default>
  <asset>
    <mesh name="proximal_mesh" file="proximal.stl"/>
    <mesh name="middle_mesh"   file="middle.stl"/>
    <mesh name="distal_mesh"   file="distal.stl"/>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0"
             width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge"
             rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8"
             width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true"
              texrepeat="5 5" reflectance="0.2"/>
  </asset>
  <worldbody>
    <light name="top" pos="0 0 2" mode="trackcom"/>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>

    <body name="gripper" pos="0 0 {flat_z:.6f}"{flat_attrib}>
    {finger_a}
    {finger_b}

    <body name="mount_block" pos="0 0 {block_z:.6f}">
      <geom type="box" size="{bhx:.6f} {bhy:.6f} {bhz:.6f}" rgba="0.25 0.25 0.28 1"
            contype="0" conaffinity="0" density="0" mass="0"/>
    </body>
    </body>
  </worldbody>
  <tendon>
{tendons}
  </tendon>
  <contact>
{excludes}
  </contact>
</mujoco>'''

    with open(XML_PATH, "w") as f:
        f.write(xml)
    return XML_PATH


if __name__ == "__main__":
    build()
    print(f"  wrote {XML_PATH}")
