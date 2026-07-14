# Variable Stiffness Pneumatic Gripper (ICRA 2026)

## Paper Summary


## Core Idea

The paper presents a **2-finger tendon-driven robotic gripper** whose finger joint stiffness can be actively varied using a **pneumatic vacuum-jamming mechanism**.

Unlike conventional robotic fingers whose stiffness remains constant throughout the grasp, this design allows the finger to continuously transition between:

- Highly compliant
- Semi-rigid
- Highly stiff

simply by changing the vacuum pressure inside a soft chamber.

The same finger therefore behaves differently depending on the manipulation task.

---

## Motivation

A rigid robotic finger

- provides accurate positioning
- supports heavy loads
- but adapts poorly to irregular objects.

A completely soft finger

- adapts well
- safely interacts with humans
- but cannot support large grasping forces.

The authors attempt to combine both advantages by creating a finger whose stiffness is **actively controllable**.

---

## Mechanical Design

### Overall Structure

The gripper consists of

- Two fingers
- Rigid links
- Revolute joints
- Tendon-driven actuation
- Pneumatic variable-stiffness joint
- Pressure regulation system

The tendon provides motion.

The vacuum system controls stiffness.

Thus,

Motion Generation ≠ Stiffness Generation.

These two functions are completely independent.

---

## Variable Stiffness Joint

The heart of the design is the variable stiffness joint.

Each finger joint contains

- rigid link
- rigid link
- soft elastic ring
- airtight chamber
- granular/soft jamming material

When atmospheric pressure exists inside

↓

the soft material deforms easily

↓

Joint behaves softly

When vacuum is applied

↓

air is removed

↓

the particles become jammed

↓

the joint becomes significantly stiffer

without changing its geometry.

This produces a continuously variable rotational stiffness.

---

## Working Principle

No Vacuum

↓

Soft chamber

↓

Easy deformation

↓

Low rotational stiffness

↓

Safe grasping

Increasing Vacuum

↓

Material compacts

↓

Higher internal friction

↓

Higher rotational stiffness

↓

Better load carrying capability

Thus

Vacuum Pressure

↓

Joint Stiffness

↓

Grasp Force Capability

---

## Tendon Mechanism

The tendon only controls

Finger Closing

Finger Opening

The tendon **does not control stiffness.**

This separation simplifies the controller because

Motion Controller

and

Stiffness Controller

become independent.

---

## Theory

Unlike classical tendon-driven fingers

where

τ = kθ

with constant stiffness,

this paper introduces

τ = f(θ,P)

where

θ = joint angle

P = vacuum pressure

Therefore

Joint stiffness

k(P)

changes continuously as pressure changes.

The stiffness is therefore no longer constant but becomes a controllable system parameter.

---

## Experimental Validation

The paper experimentally validates the proposed mechanism through several tests.

---

### 1. Pressure vs Joint Stiffness

Objective

Determine how stiffness changes with vacuum pressure.

Observation

Increasing vacuum

↓

Higher rotational stiffness

↓

Joint becomes increasingly rigid.

This validates the variable stiffness concept.

---

### 2. Torque-Angle Characteristics

The authors apply external torques to the joint and measure

Joint Angle

vs

Applied Torque.

Result

Different pressure levels produce different torque-angle curves.

Hence

Pressure directly changes rotational stiffness.

---

### 3. Pressure vs Bending

The finger is actuated under different vacuum levels.

Observation

Low pressure

↓

Finger bends easily.

High pressure

↓

Finger resists bending.

This experimentally demonstrates controllable compliance.

---

### 4. Adaptive Grasping

Objects having different stiffnesses are grasped.

Soft object

↓

Low stiffness finger

Rigid object

↓

High stiffness finger

This demonstrates adaptive grasp behavior.

---

### 5. Object Stiffness Estimation

One of the paper's major contributions.

Using

- finger deformation
- applied pressure
- joint motion

the system estimates

Object stiffness

without requiring additional external sensing.

The finger itself acts as a sensing mechanism.

---

## Main Contributions

The paper introduces

- Pneumatic variable stiffness joint
- Independent motion and stiffness control
- Vacuum-jamming based stiffness modulation
- Object stiffness estimation
- Experimental validation of stiffness adaptation

---

## Outcomes

The proposed design achieves

✓ Continuously variable joint stiffness

✓ Stable grasping over different object types

✓ Improved adaptability

✓ Object stiffness estimation

✓ Lightweight implementation

✓ Safe compliant manipulation

without changing the tendon transmission.

---

## Advantages

- Simple mechanism
- Independent stiffness control
- Passive safety
- Improved grasp stability
- Better adaptation to unknown objects
- Suitable for delicate manipulation

---

## Limitations

The work does not

- derive a complete analytical dynamics model
- optimize stiffness automatically
- perform reinforcement learning
- benchmark against extensive grasp taxonomies
- evaluate very large payload capacities

The primary focus remains on validating the proposed mechanical concept.

---

## Relation to Our Project

This paper is directly relevant to our variable stiffness tendon-driven hand.

Current repository:

Fixed stiffness

↓

Single joint stiffness values defined inside config.py

↓

Stiffness changed manually

Our proposed framework:

Variable stiffness sliders

↓

Runtime stiffness modification

↓

Continuous stiffness variation

↓

Interactive tendon displacement control

↓

Performance visualization

↓

(Future reinforcement learning.)

---

## Ideas We Can Incorporate

### Runtime Variable Stiffness

Replace Fixed stiffness constants with continuously adjustable runtime parameters.

---

### Independent Motion and Stiffness

Similar to this paper,

keep Tendon displacement independent from Joint stiffness.

This makes the simulation much closer to future hardware.

---

### Performance Graphs

Plot

- Joint angle vs tendon displacement
- Tendon force vs displacement
- Joint torque vs stiffness
- Finger trajectory
- Stiffness ratio history

---

### Variable Stiffness Experiments

Investigate

- grasp stability
- payload
- tendon force
- energy consumption
- contact force

for different stiffness distributions.

---

### RL Extension(In future)

Eventually allow reinforcement learning to determine

Optimal stiffness

during grasping.

The RL policy learns

Object

↓

Required stiffness

↓

Optimal grasp.

---

## Personal Takeaways

This paper demonstrates that a novel **mechanical idea**, supported by clear experimental validation, is sufficient for publication at ICRA.

Its biggest contribution is **not a new control algorithm**, but introducing a practical variable-stiffness joint and experimentally proving its advantages.

For our research, the most valuable lesson is the separation between

Motion Generation

and

Stiffness Regulation.

Our simulation can naturally extend this idea by allowing runtime stiffness control, systematic benchmarking, and eventually intelligent stiffness optimization through learning-based approaches.