# CRAFT: Compliant Robotic Hand with Adaptive Finger Tips
*A Hybrid Hard-Soft Robotic Hand for Dexterous Manipulation*

---

## Objective

The CRAFT hand aims to bridge the gap between **fully rigid robotic hands** and **fully soft robotic hands** by introducing a **hybrid rigid-soft architecture**.

The primary goal is to achieve:

- High grasp adaptability
- Mechanical compliance
- Precision manipulation
- Low manufacturing cost
- Ease of fabrication using FDM 3D printing

---

## Motivation

### Problems with Fully Rigid Hands

Examples:
- LEAP Hand
- Allegro Hand
- Shadow Hand

Advantages:
- Accurate kinematics
- High precision
- High repeatability

Limitations:
- Poor passive compliance
- Unsafe interaction with fragile objects
- Requires complex impedance/control algorithms

---

### Problems with Fully Soft Hands

Advantages:
- Naturally compliant
- Safe interaction
- Excellent object conformity

Limitations:
- Poor positional accuracy
- Difficult mathematical modelling
- Lower repeatability
- Reduced load capacity

---

## Design Philosophy

Instead of making the entire hand rigid or soft,

CRAFT introduces:

> **Rigid links + Soft compliant joints**

Compliance is provided **only where it is beneficial**, while rigid links preserve structural integrity and kinematic accuracy.

---

## Mechanical Architecture

### Overall Hand

- 15 Actuators
- 20 Degrees of Freedom
- 15 Active DOFs
- 5 Passive DOFs

The hand is intentionally **underactuated** to reduce:

- Cost
- Weight
- Mechanical complexity

---

## Finger Design

Each finger consists of:

- MCP Joint
- PIP Joint
- DIP Joint

---

### MCP Joint

Provides:

- Flexion / Extension
- Abduction / Adduction

Implemented using a **spherical pop-in joint**, inspired by the ORCA Hand.

---

### PIP & DIP Joints

Features:

- Mechanically coupled
- Driven using a **single tendon**

Advantages:

- Fewer actuators
- Lower weight
- Simpler control
- More natural grasping motion

Trade-off:

- PIP and DIP cannot be independently controlled.

---

## Hybrid Material Selection

### PLA (Rigid Components)

Used for:

- Finger links
- Structural components

Purpose:

- Maintain geometry
- Efficient force transmission
- High stiffness

---

### TPU (Soft Components)

Used for:

- PIP joints
- DIP joints
- Compliant interfaces

Purpose:

- Passive compliance
- Shock absorption
- Increased contact area
- Safer interaction

---

## Rolling Contact Inspired Joint

Instead of conventional pin joints,

CRAFT employs rolling-contact-inspired compliant joints.

Advantages:

- Lower stress concentration
- Reduced backlash
- Improved compliance
- Better durability

**Note:** The rolling motion is assisted by TPU structures that maintain contact between rigid bodies.

---

## Tendon Routing

Finger tendons are routed using guide pins.

Purpose:

- Maintain tendon path
- Reduce friction
- Maintain consistent tendon moment arm

---

## Ratchet Spool

The tendon tension is adjusted using a ratchet spool.

Purpose:

- Pretension tendons
- Prevent tendon loosening
- Simplify assembly and maintenance

---

## Thumb Design

The thumb includes:

- CMC Joint
- MCP Joint
- IP Joint

allowing improved opposition and dexterity.

---

## Experimental Evaluation

The paper evaluates the hand using:

### 1. Structural Capability

Ability to withstand external loads.

---

### 2. Precision

Ability to accurately achieve desired finger positions.

---

### 3. Repeatability

Consistency across repeated grasping motions.

---

### 4. Grasp Capability

Evaluation using:

- Small objects
- Large objects
- Irregular objects
- Fragile objects

---

## Comparison

Primary comparison is made with the **LEAP Hand**.

Reported improvements include:

- Better passive compliance
- Improved impact tolerance
- Lower fabrication cost
- Easier manufacturing
- Comparable grasping performance

---

## Major Contributions

- Hybrid hard-soft robotic finger
- Rolling-contact-inspired compliant joints
- Underactuated tendon-driven architecture
- Low-cost fabrication using FDM printing
- Modular mechanical design
- Open-source hardware

---

## Limitations

- Passive compliance only
- Fixed joint stiffness
- No adaptive stiffness control
- Coupled PIP/DIP reduces dexterity
- Mechanical properties depend on TPU characteristics

---

## Relevance to Our Variable Stiffness Project

CRAFT demonstrates that introducing compliance at the joints significantly improves grasp adaptability.

However, the joint stiffness remains fixed after fabrication.

Our project extends this idea by replacing **fixed passive compliance** with **runtime-adjustable joint stiffness**, enabling:

- Task-specific stiffness
- Adaptive grasp behaviour
- Variable stiffness actuation
- Dynamic compliance control

---

## Key Takeaways

- Compliance should exist only where it is beneficial.
- Underactuation is an effective design trade-off.
- Tendon routing significantly affects force transmission and repeatability.
- Hybrid rigid-soft construction provides an excellent balance between precision and adaptability.
- CRAFT is primarily an engineering integration of proven concepts (ORCA-inspired compliance + LEAP-level dexterity) into a practical, low-cost robotic hand.

---

## One-Line Summary

> **CRAFT is a low-cost, hybrid rigid-soft underactuated robotic hand that combines localized passive compliance with tendon-driven actuation to achieve dexterous, repeatable, and manufacturable grasping, while leaving adaptive stiffness as an open avenue for future research.**