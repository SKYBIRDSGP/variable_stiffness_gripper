#!/usr/bin/env python3
"""
build.py — Franka Panda + 2-Finger Tendon Gripper

Combines the Franka arm (from panda.xml) with the validated tendon-driven
2-finger gripper (from gripper/build_gripper.py). Replaces the stock Franka
hand with our underactuated gripper at 100 mm finger separation.
"""

import os
import sys
import math
import json
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation as Rot

_HERE = os.path.dirname(os.path.abspath(__file__))
_FRANKA_DIR = os.path.join(_HERE, "..")
_REPO_ROOT = os.path.join(_FRANKA_DIR, "..")
_HIFI_MESHES = os.path.join(_REPO_ROOT, "high_fidelity", "meshes")

if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)
import config

MM = 1e-3
JOINT_NAMES = ("mcp", "pip", "dip")
SEPARATION = 0.100  # 100 mm between finger centres
STIFFNESS_VAL = 1.0


def _load_params():
    params_path = os.path.join(_REPO_ROOT, "high_fidelity", "params.json")
    with open(params_path) as f:
        return json.load(f)


def _v3(a):
    return f"{a[0]:.6f} {a[1]:.6f} {a[2]:.6f}"


def _q4(q):
    return f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}"


def _fi(P, nm):
    I = np.array(P["inertial"][nm]["I"])
    return (f"{I[0,0]:.6e} {I[1,1]:.6e} {I[2,2]:.6e} "
            f"{I[0,1]:.6e} {I[0,2]:.6e} {I[1,2]:.6e}")


def _com(P, nm):
    c = P["inertial"][nm]["com_body"]
    return f"{c[0]:.6f} {c[1]:.6f} {c[2]:.6f}"


_MASS_MAP = {"proximal": config.PROXIMAL_MASS,
             "middle": config.MIDDLE_MASS,
             "distal": config.DISTAL_MASS}


def _mass(P, nm):
    return _MASS_MAP[nm]


def _rng(r):
    return f"{r[0] * math.pi / 180:.5f} {r[1] * math.pi / 180:.5f}"


def _quat_wxyz(R):
    x, y, z, w = R.as_quat()
    return w, x, y, z


def _finger_body_xml(prefix, base_pos, base_quat, P):
    mcp = np.array(config.MCP_CENTER)
    pip = np.array(config.PIP_CENTER)
    dip = np.array(config.DIP_CENTER)
    tip = np.array(config.TIP_POINT)

    pos_prox = mcp * MM
    pos_mid = (pip - mcp) * MM
    pos_dist = (dip - pip) * MM
    tip_local = (tip - dip) * MM

    dp = (config.MCP_DAMPING, config.PIP_DAMPING, config.DIP_DAMPING)
    rg = (config.MCP_RANGE, config.PIP_RANGE, config.DIP_RANGE)
    j = [f"{prefix}_{n}" for n in JOINT_NAMES]

    bl_h = config.BASE_LINK_LENGTH_MM / 2 * MM
    bl_cx = (config.MCP_CENTER[0] + config.BASE_LINK_INNER_GAP_MM
             + config.BASE_LINK_LENGTH_MM / 2) * MM

    return f'''
    <body name="{prefix}_base" pos="{_v3(base_pos)}" quat="{_q4(base_quat)}">
      <geom type="box" size="{bl_h:.6f} 0.010 0.0075" pos="{bl_cx:.6f} 0 0"
            rgba="0.30 0.30 0.33 1" contype="0" conaffinity="0" density="0" mass="0"/>

      <body name="{prefix}_proximal" pos="{_v3(pos_prox)}">
        <joint name="{j[0]}" stiffness="{STIFFNESS_VAL}" damping="{dp[0]}" range="{_rng(rg[0])}"/>
        <inertial pos="{_com(P, 'proximal')}" mass="{_mass(P, 'proximal'):.6f}" fullinertia="{_fi(P, 'proximal')}"/>
        <geom class="phalanx" mesh="proximal_mesh"/>

        <body name="{prefix}_middle" pos="{_v3(pos_mid)}">
          <joint name="{j[1]}" stiffness="{STIFFNESS_VAL}" damping="{dp[1]}" range="{_rng(rg[1])}"/>
          <inertial pos="{_com(P, 'middle')}" mass="{_mass(P, 'middle'):.6f}" fullinertia="{_fi(P, 'middle')}"/>
          <geom class="phalanx" mesh="middle_mesh"/>

          <body name="{prefix}_distal" pos="{_v3(pos_dist)}">
            <joint name="{j[2]}" stiffness="{STIFFNESS_VAL}" damping="{dp[2]}" range="{_rng(rg[2])}"/>
            <inertial pos="{_com(P, 'distal')}" mass="{_mass(P, 'distal'):.6f}" fullinertia="{_fi(P, 'distal')}"/>
            <geom class="phalanx" mesh="distal_mesh"/>
            <site name="{prefix}_tip" pos="{_v3(tip_local)}" size="0.001" rgba="0.2 0.7 0.95 1"/>
          </body>
        </body>
      </body>
    </body>'''


def build_combined_xml():
    P = _load_params()
    arm = config.SHEATH_MOMENT_ARM

    # ── Gripper base poses (mounted in the hand body frame) ──
    R_A = Rot.from_euler("y", 90, degrees=True)
    R_B = Rot.from_euler("z", 180, degrees=True) * R_A
    qA, qB = _quat_wxyz(R_A), _quat_wxyz(R_B)
    posA = (0.0, +SEPARATION / 2.0, 0.0)
    posB = (0.0, -SEPARATION / 2.0, 0.0)

    finger_a = _finger_body_xml("a", posA, qA, P)
    finger_b = _finger_body_xml("b", posB, qB, P)

    tendons = (f'    <fixed name="a_flexor" stiffness="{config.TENDON_STIFFNESS}" '
               f'damping="{config.TENDON_DAMPING}" springlength="-1">\n'
               + '\n'.join(f'      <joint joint="a_{n}" coef="{-arm}"/>'
                           for n in JOINT_NAMES)
               + '\n    </fixed>\n'
               f'    <fixed name="b_flexor" stiffness="{config.TENDON_STIFFNESS}" '
               f'damping="{config.TENDON_DAMPING}" springlength="-1">\n'
               + '\n'.join(f'      <joint joint="b_{n}" coef="{-arm}"/>'
                           for n in JOINT_NAMES)
               + '\n    </fixed>')

    excludes = '\n'.join(
        f'    <exclude body1="{p}_{a}" body2="{p}_{b}"/>'
        for p in ("a", "b")
        for a, b in (("proximal", "middle"),
                     ("middle", "distal"),
                     ("proximal", "distal")))

    # Mesh dir relative to the generated XML location
    meshdir = os.path.relpath(_HIFI_MESHES, _HERE)

    hand_body = f'''
    <!-- ===== gripper replaces the stock Franka hand ===== -->
    <body name="hand" pos="0 0 0.107" quat="0.9238795 0 0 -0.3826834">
      <inertial mass="0.73" pos="-0.010000 0.000000 0.030000"
                diaginertia="0.001000 0.002500 0.001700"/>
      {finger_a}
      {finger_b}
    </body>'''

    xml = f'''<mujoco model="franka_tendon_panda">
  <compiler angle="radian" meshdir="{meshdir}" autolimits="true"/>

  <option integrator="implicitfast" timestep="{config.GRIPPER_TIMESTEP}"
          cone="{config.GRIPPER_FRICTION_CONE}" impratio="{config.GRIPPER_IMPRATIO}"/>

  <default>
    <default class="panda">
      <material specular="0.5" shininess="0.25"/>
      <joint armature="0.1" damping="1" axis="0 0 1" range="-2.8973 2.8973"/>
      <general dyntype="none" biastype="affine" ctrlrange="-2.8973 2.8973" forcerange="-87 87"/>
      <default class="visual">
        <geom type="mesh" contype="0" conaffinity="0" group="2"/>
      </default>
      <default class="collision">
        <geom type="mesh" group="3"/>
      </default>
    </default>

    <!-- Gripper phalanx contact defaults -->
    <geom condim="{config.GRIPPER_CONTACT_CONDIM}" friction="{config.GRIPPER_CONTACT_FRICTION}"
          solref="{config.GRIPPER_CONTACT_SOLREF}" solimp="{config.GRIPPER_CONTACT_SOLIMP}"
          density="0"/>
    <joint type="hinge" axis="0 0 1" damping="0.08" limited="true"
           solreflimit="{config.LIMIT_SOLREF}" solimplimit="{config.LIMIT_SOLIMP}"/>
    <default class="phalanx">
      <geom type="mesh" rgba="0.78 0.80 0.85 1" contype="1" conaffinity="1"/>
    </default>
    <site group="3"/>
  </default>

  <asset>
    <!-- Franka collision meshes -->
    <mesh name="link0_c" file="{os.path.join(_FRANKA_DIR, 'assets/link0.stl')}"/>
    <mesh name="link1_c" file="{os.path.join(_FRANKA_DIR, 'assets/link1.stl')}"/>
    <mesh name="link2_c" file="{os.path.join(_FRANKA_DIR, 'assets/link2.stl')}"/>
    <mesh name="link3_c" file="{os.path.join(_FRANKA_DIR, 'assets/link3.stl')}"/>
    <mesh name="link4_c" file="{os.path.join(_FRANKA_DIR, 'assets/link4.stl')}"/>
    <mesh name="link5_c0" file="{os.path.join(_FRANKA_DIR, 'assets/link5_collision_0.obj')}"/>
    <mesh name="link5_c1" file="{os.path.join(_FRANKA_DIR, 'assets/link5_collision_1.obj')}"/>
    <mesh name="link5_c2" file="{os.path.join(_FRANKA_DIR, 'assets/link5_collision_2.obj')}"/>
    <mesh name="link6_c" file="{os.path.join(_FRANKA_DIR, 'assets/link6.stl')}"/>
    <mesh name="link7_c" file="{os.path.join(_FRANKA_DIR, 'assets/link7.stl')}"/>
    <mesh name="hand_c" file="{os.path.join(_FRANKA_DIR, 'assets/hand.stl')}"/>

    <!-- Franka visual meshes -->
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link0_0.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link0_1.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link0_2.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link0_3.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link0_4.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link0_5.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link0_7.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link0_8.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link0_9.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link0_10.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link0_11.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link1.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link2.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link3_0.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link3_1.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link3_2.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link3_3.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link4_0.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link4_1.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link4_2.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link4_3.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link5_0.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link5_1.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link5_2.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link6_0.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link6_1.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link6_2.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link6_3.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link6_4.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link6_5.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link6_6.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link6_7.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link6_8.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link6_9.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link6_10.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link6_11.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link6_12.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link6_13.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link6_14.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link6_15.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link6_16.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link7_0.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link7_1.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link7_2.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link7_3.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link7_4.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link7_5.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link7_6.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/link7_7.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/hand_0.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/hand_1.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/hand_2.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/hand_3.obj')}"/>
    <mesh file="{os.path.join(_FRANKA_DIR, 'assets/hand_4.obj')}"/>

    <!-- Gripper phalanx meshes -->
    <mesh name="proximal_mesh" file="proximal.stl"/>
    <mesh name="middle_mesh"   file="middle.stl"/>
    <mesh name="distal_mesh"   file="distal.stl"/>

    <!-- Materials & textures -->
    <material class="panda" name="white" rgba="0.9 0.9 0.95 1"/>
    <material class="panda" name="off_white" rgba="0.902 0.922 0.929 1"/>
    <material class="panda" name="black" rgba="0.25 0.25 0.25 1"/>
    <material class="panda" name="green" rgba="0 1 0 1"/>
    <material class="panda" name="light_blue" rgba="0.039 0.541 0.780 1"/>
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

    <!-- Franka arm link0 (base, fixed to world) -->
    <body name="link0" childclass="panda">
      <inertial mass="0.629769" pos="-0.041018 -0.00014 0.049974"
        fullinertia="0.00315 0.00388 0.004285 8.2904e-7 0.00015 8.2299e-6"/>
      <geom mesh="link0_0" material="off_white" class="visual"/>
      <geom mesh="link0_1" material="black" class="visual"/>
      <geom mesh="link0_2" material="off_white" class="visual"/>
      <geom mesh="link0_3" material="black" class="visual"/>
      <geom mesh="link0_4" material="off_white" class="visual"/>
      <geom mesh="link0_5" material="black" class="visual"/>
      <geom mesh="link0_7" material="white" class="visual"/>
      <geom mesh="link0_8" material="white" class="visual"/>
      <geom mesh="link0_9" material="black" class="visual"/>
      <geom mesh="link0_10" material="off_white" class="visual"/>
      <geom mesh="link0_11" material="white" class="visual"/>
      <geom mesh="link0_c" class="collision"/>

      <!-- link1 -->
      <body name="link1" pos="0 0 0.333">
        <joint name="joint1"/>
        <inertial mass="4.970684" pos="0.003875 0.002081 -0.04762"
          fullinertia="0.70337 0.70661 0.0091170 -0.00013900 0.0067720 0.019169"/>
        <geom material="white" mesh="link1" class="visual"/>
        <geom mesh="link1_c" class="collision"/>

        <!-- link2 (quat -90° X) -->
        <body name="link2" quat="1 -1 0 0">
          <joint name="joint2" range="-1.7628 1.7628"/>
          <inertial mass="0.646926" pos="-0.003141 -0.02872 0.003495"
            fullinertia="0.0079620 2.8110e-2 2.5995e-2 -3.925e-3 1.0254e-2 7.04e-4"/>
          <geom material="white" mesh="link2" class="visual"/>
          <geom mesh="link2_c" class="collision"/>

          <!-- link3 -->
          <body name="link3" pos="0 -0.316 0" quat="1 1 0 0">
            <joint name="joint3"/>
            <inertial mass="3.228604" pos="2.7518e-2 3.9252e-2 -6.6502e-2"
              fullinertia="3.7242e-2 3.6155e-2 1.083e-2 -4.761e-3 -1.1396e-2 -1.2805e-2"/>
            <geom mesh="link3_0" material="white" class="visual"/>
            <geom mesh="link3_1" material="white" class="visual"/>
            <geom mesh="link3_2" material="white" class="visual"/>
            <geom mesh="link3_3" material="black" class="visual"/>
            <geom mesh="link3_c" class="collision"/>

            <!-- link4 -->
            <body name="link4" pos="0.0825 0 0" quat="1 1 0 0">
              <joint name="joint4" range="-3.0718 -0.0698"/>
              <inertial mass="3.587895" pos="-5.317e-2 1.04419e-1 2.7454e-2"
                fullinertia="2.5853e-2 1.9552e-2 2.8323e-2 7.796e-3 -1.332e-3 8.641e-3"/>
              <geom mesh="link4_0" material="white" class="visual"/>
              <geom mesh="link4_1" material="white" class="visual"/>
              <geom mesh="link4_2" material="black" class="visual"/>
              <geom mesh="link4_3" material="white" class="visual"/>
              <geom mesh="link4_c" class="collision"/>

              <!-- link5 -->
              <body name="link5" pos="-0.0825 0.384 0" quat="1 -1 0 0">
                <joint name="joint5"/>
                <inertial mass="1.225946" pos="-1.1953e-2 4.1065e-2 -3.8437e-2"
                  fullinertia="3.5549e-2 2.9474e-2 8.627e-3 -2.117e-3 -4.037e-3 2.29e-4"/>
                <geom mesh="link5_0" material="black" class="visual"/>
                <geom mesh="link5_1" material="white" class="visual"/>
                <geom mesh="link5_2" material="white" class="visual"/>
                <geom mesh="link5_c0" class="collision"/>
                <geom mesh="link5_c1" class="collision"/>
                <geom mesh="link5_c2" class="collision"/>

                <!-- link6 -->
                <body name="link6" quat="1 1 0 0">
                  <joint name="joint6" range="-0.0175 3.7525"/>
                  <inertial mass="1.666555" pos="6.0149e-2 -1.4117e-2 -1.0517e-2"
                    fullinertia="1.964e-3 4.354e-3 5.433e-3 1.09e-4 -1.158e-3 3.41e-4"/>
                  <geom mesh="link6_0" material="off_white" class="visual"/>
                  <geom mesh="link6_1" material="white" class="visual"/>
                  <geom mesh="link6_2" material="black" class="visual"/>
                  <geom mesh="link6_3" material="white" class="visual"/>
                  <geom mesh="link6_4" material="white" class="visual"/>
                  <geom mesh="link6_5" material="white" class="visual"/>
                  <geom mesh="link6_6" material="white" class="visual"/>
                  <geom mesh="link6_7" material="light_blue" class="visual"/>
                  <geom mesh="link6_8" material="light_blue" class="visual"/>
                  <geom mesh="link6_9" material="black" class="visual"/>
                  <geom mesh="link6_10" material="black" class="visual"/>
                  <geom mesh="link6_11" material="white" class="visual"/>
                  <geom mesh="link6_12" material="green" class="visual"/>
                  <geom mesh="link6_13" material="white" class="visual"/>
                  <geom mesh="link6_14" material="black" class="visual"/>
                  <geom mesh="link6_15" material="black" class="visual"/>
                  <geom mesh="link6_16" material="white" class="visual"/>
                  <geom mesh="link6_c" class="collision"/>

                  <!-- link7 (flange) -->
                  <body name="link7" pos="0.088 0 0" quat="1 1 0 0">
                    <joint name="joint7"/>
                    <inertial mass="7.35522e-01" pos="1.0517e-2 -4.252e-3 6.1597e-2"
                      fullinertia="1.2516e-2 1.0027e-2 4.815e-3 -4.28e-4 -1.196e-3 -7.41e-4"/>
                    <geom mesh="link7_0" material="white" class="visual"/>
                    <geom mesh="link7_1" material="black" class="visual"/>
                    <geom mesh="link7_2" material="black" class="visual"/>
                    <geom mesh="link7_3" material="black" class="visual"/>
                    <geom mesh="link7_4" material="black" class="visual"/>
                    <geom mesh="link7_5" material="black" class="visual"/>
                    <geom mesh="link7_6" material="black" class="visual"/>
                    <geom mesh="link7_7" material="white" class="visual"/>
                    <geom mesh="link7_c" class="collision"/>

                    {hand_body}
                  </body>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <tendon>
{tendons}
  </tendon>

  <actuator>
    <general class="panda" name="actuator1" joint="joint1" gainprm="4500" biasprm="0 -4500 -450"/>
    <general class="panda" name="actuator2" joint="joint2" gainprm="4500" biasprm="0 -4500 -450"
      ctrlrange="-1.7628 1.7628"/>
    <general class="panda" name="actuator3" joint="joint3" gainprm="3500" biasprm="0 -3500 -350"/>
    <general class="panda" name="actuator4" joint="joint4" gainprm="3500" biasprm="0 -3500 -350"
      ctrlrange="-3.0718 -0.0698"/>
    <general class="panda" name="actuator5" joint="joint5" gainprm="2000" biasprm="0 -2000 -200" forcerange="-12 12"/>
    <general class="panda" name="actuator6" joint="joint6" gainprm="2000" biasprm="0 -2000 -200" forcerange="-12 12"
      ctrlrange="-0.0175 3.7525"/>
    <general class="panda" name="actuator7" joint="joint7" gainprm="2000" biasprm="0 -2000 -200" forcerange="-12 12"/>

  </actuator>

  <contact>
{excludes}
  </contact>

  <keyframe>
    <key name="home" qpos="0 0 0 -1.57079 0 1.57079 -0.7853 0 0 0 0 0 0"
         ctrl="0 0 0 -1.57079 0 1.57079 -0.7853"/>
  </keyframe>
</mujoco>'''

    out_path = os.path.join(_HERE, "franka_tendon.xml")
    with open(out_path, "w") as f:
        f.write(xml)
    print(f"  wrote {out_path}")
    return out_path


def build(*, force=True):
    out_path = os.path.join(_HERE, "franka_tendon.xml")
    if os.path.exists(out_path) and not force:
        return out_path
    return build_combined_xml()


if __name__ == "__main__":
    build()
