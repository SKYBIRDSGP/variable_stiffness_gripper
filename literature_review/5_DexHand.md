# 📄 Paper Summary – DexHand-021: A Reconfigurable Dexterous Hand with Proprioceptive Compliant Control

---

## 1. Objective of the Paper

DexHand-021 aims to develop a lightweight, highly dexterous robotic hand that combines:

- High dexterity
- Modular mechanical design
- Tendon-driven actuation
- Proprioceptive force estimation
- Compliant manipulation

Unlike conventional robotic hands that rely heavily on external force sensors, DexHand-021 estimates joint torques internally and uses them for compliant control.

---

## 2. Human Hand Inspiration

The human hand possesses:

- 22 DOFs
- 19 Bones
- 29 Muscles

Replicating every muscle is impractical due to:

- Space limitations
- Weight
- Cost
- Motor packaging

Therefore, the authors designed a robotic hand having:

- **12 Brushless Hollow Cup Motors**
- **19 Degrees of Freedom**

while maintaining human-like grasping capability.

---

## 3. Mechanical Design

### 3.1 Finger DOFs

Each finger contains:

- MCP Flexion
- MCP Abduction
- PIP
- DIP (Underactuated)

Thumb contains:

- CMR
- CMP
- MPP
- DIP

Total:

**19 DOFs driven using only 12 motors.**

---

### 3.2 Underactuated Finger

Instead of independently actuating both joints,

- One tendon drives
  - PIP
  - DIP

This reduces:

- Number of motors
- Weight
- Complexity

while still producing human-like curling.

---

### 3.3 Artificial Muscle

The paper introduces a modular **Artificial Muscle Unit**.

It is **not** a soft pneumatic muscle (such as McKibben muscles).

Instead, it is an electromechanical module consisting of:

- Hollow cup DC Motor
- Planetary Gearbox
- Worm Gear
- Capstan / Pulley
- Tendon (Tungsten Cable)
- Passive Spring

This entire module behaves similarly to a biological muscle.

Each unit:

- consumes approximately **6 W**
- produces approximately **150 N continuous tendon force**

---

### 3.4 Modular Design

The hand is divided into:

- Thumb Module
- Four-Finger Module
- MCP Module

This makes:

- assembly easier
- maintenance easier
- replacement simpler

---

## 4. Hill Muscle Model Inspiration

The biological muscle follows the Hill Muscle Model consisting of:

- Contractile Element
- Parallel Elastic Element
- Series Elastic Element

Instead of replicating muscles physically,

the authors mathematically reproduce the same behavior using motors and springs.

The tendon force is modeled as

Fₘ = Current
    + Position Error
    + Velocity Error
    + Spring Force

Thus, the actuator behaves similarly to a biological muscle.

---

## 5. Sensors Used

To estimate joint torque, the hand uses:

- Motor Current
- Motor Position
- Motor Velocity
- Joint Position
- Joint Velocity
- Temperature
- Hall Sensors
- Tactile Sensors
- Force/Torque Sensors

These measurements together form the proprioceptive feedback of the hand.

---

## 6. Nonlinear Torque Estimation

The relationship between:

- Motor Current
- Joint Motion
- Temperature

and

Joint Torque

is highly nonlinear.

Therefore, instead of deriving analytical equations,

the paper formulates the problem as a machine learning regression task.

---

### Inputs (X)

- Motor Current
- Motor Position
- Motor Velocity
- Joint Position
- Joint Velocity
- Temperature

---

### Output (Y)

Estimated Joint Torque

---

## 7. Gaussian Process Regression (GPR)

The authors employ **Gaussian Process Regression (GPR)** to learn the nonlinear mapping

X → Joint Torque

Reasons for choosing GPR:

- Excellent nonlinear modeling capability
- Works well with small datasets
- Provides prediction uncertainty
- Resistant to overfitting

The kernel consists of:

- Radial Basis Function (RBF)
- White Noise Kernel

which together model

- nonlinear behavior
- measurement noise

respectively.

---

## 8. Proprioceptive Torque Estimation

Instead of measuring joint torque directly,

the learned GPR model estimates it from the sensor data.

Thus,

Estimated Torque

≈

Actual Joint Torque

This estimated torque serves as the hand's internal perception of interaction forces.

---

## 9. Impedance (Compliant) Control

Once joint torque is estimated,

the controller computes

External Torque

=

Desired Torque

−

Estimated Torque

Instead of rigidly resisting disturbances,

the controller behaves like a virtual

- Spring
- Damper
- Mass

This is represented using the impedance equation

τ = MΔq¨ + BΔq˙ + KΔq

where

M → virtual inertia

B → virtual damping

K → virtual stiffness

Thus the finger naturally yields when external forces are applied.

---

## 10. Position Control

The impedance controller generates a new desired joint position.

A standard PID controller then drives the motors toward this position.

Overall control pipeline:

Desired Torque

↓

Artificial Muscle

↓

Finger Motion

↓

Sensor Measurements

↓

Gaussian Process Regression

↓

Estimated Joint Torque

↓

External Torque Estimation

↓

Impedance Controller

↓

Desired Joint Position

↓

PID Controller

↓

Motor Current

---

## 11. Major Contributions

- 19 DOF robotic hand using only 12 motors
- Modular tendon-driven architecture
- Artificial muscle module inspired by Hill Muscle Model
- Underactuated DIP joints
- Internal torque estimation using Gaussian Process Regression
- Proprioceptive compliant control without relying solely on dedicated torque sensors
- Stable compliant grasping of unknown objects
- Human-safe interaction through impedance control

---

## 12. Key Takeaways

Unlike previous tendon-driven hands that mainly focus on mechanical design,

DexHand-021 integrates:

- Mechanical Design
- Machine Learning
- Torque Estimation
- Proprioception
- Impedance Control

into a single robotic hand.

The paper demonstrates how internal sensing combined with learned torque estimation enables compliant manipulation without requiring expensive torque sensors at every joint.

---

## 13. Inspiration for Our Variable Stiffness Hand

This paper provides valuable ideas beyond the mechanical design.

Potential concepts applicable to our project include:

- Torque estimation from internal sensors
- Proprioceptive feedback for tendon-driven fingers
- Variable virtual stiffness through impedance control
- Learning-based force estimation
- Integration of compliant control with tendon-driven underactuated fingers

These concepts can naturally complement our ongoing work on variable stiffness tendon-driven robotic fingers.
