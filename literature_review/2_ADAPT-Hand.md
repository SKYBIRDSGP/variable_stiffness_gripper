# Robotic Hand Research Notebook

# Paper 02

## The ADAPT Hand: Design and Open-Loop Grasping of a Highly Adaptable Anthropomorphic Robot Hand

---

# 1. Objective

The primary objective of this work is to design an anthropomorphic robotic hand capable of robustly grasping a wide variety of objects while maintaining a relatively simple control architecture.

Unlike many robotic hands that rely heavily on sensors, complex control algorithms, and fully actuated joints, the ADAPT Hand demonstrates that intelligent mechanical design can significantly simplify the grasping problem. The authors achieve this through tendon-driven actuation, underactuation, distributed compliance, and passive adaptation.

The philosophy of the paper can be summarized as:

> *Instead of making the controller more intelligent, make the mechanism more intelligent.*

---

# 2. Motivation

Most robotic hands generally belong to one of two categories.

### Fully Actuated Robotic Hands

These hands provide excellent dexterity because each joint can be independently controlled. However, they suffer from several drawbacks:

- Large number of motors
- High cost
- Increased weight
- Complex control algorithms
- Difficult maintenance

### Soft Robotic Hands

Soft robotic hands naturally adapt to different object shapes and are safe when interacting with humans.

However, they often sacrifice:

- Payload capacity
- Precision
- Grasp stability

The ADAPT Hand attempts to bridge these two worlds by introducing compliance only where it is beneficial while retaining the advantages of tendon-driven rigid mechanisms.

---

# 3. Mechanical Design

The hand consists of:

- Four identical fingers
- One anthropomorphic thumb
- Tendon-driven transmission
- Series elastic elements
- Soft fingertips
- Compliant wrist

The overall architecture follows the human hand rather than attempting to perfectly replicate human anatomy.

The authors intentionally simplify several joints while preserving the motions required for common grasping tasks.

---

# 4. Tendon-Driven Mechanism

The ADAPT Hand uses remotely located electric motors that transmit force through tendons.

Each finger is driven using separate tendon paths responsible for different joint motions.

Instead of mounting motors directly inside the fingers, the motors remain inside the palm, reducing finger inertia and allowing the fingers to remain lightweight.

Advantages of tendon-driven transmission include:

- Lower moving mass
- Compact finger design
- Easier routing of actuators
- Human-inspired actuation
- Reduced inertia during motion

---

# 5. Underactuation

One of the most important concepts introduced in this paper is **underactuation**.

The robotic hand possesses more mechanical degrees of freedom than actuators.

Rather than assigning one motor to every joint, multiple joints are mechanically coupled using tendons.

This provides several advantages:

- Reduced actuator count
- Lower overall weight
- Lower manufacturing cost
- Simpler control architecture
- Passive adaptation to object geometry

The resulting grasp configuration is determined not only by motor commands but also by the interaction between the fingers and the object.

---

# 6. Compliance Strategy

Rather than making only the fingers compliant, the authors intentionally distribute compliance throughout the entire hand.

## Fingertip Compliance

The fingertips are covered using compliant material that increases the contact area between the object and the finger.

Benefits include:

- Higher friction
- Lower contact pressure
- Safer interaction
- Better grasp stability

---

## Finger Compliance

Series elastic elements within the tendon transmission allow the fingers to naturally adapt to object geometry.

Instead of requiring explicit force control, the fingers mechanically conform to the object.

---

## Wrist Compliance

The wrist also contains compliance.

Small positioning errors during grasping are absorbed mechanically rather than causing grasp failure.

This significantly improves robustness when object locations are uncertain.

---

# 7. Variable Stiffness

An important observation while reading this paper is that the ADAPT Hand **does not implement active variable stiffness**.

Instead, it relies on **fixed mechanical compliance**.

The compliance originates from:

- Series elastic elements
- Soft fingertips
- Flexible wrist

Unlike the previous TUM paper, the stiffness cannot be actively changed during grasping.

Therefore:

- Compliance ✔
- Active Variable Stiffness ✘

---

# 8. Thumb Design

The thumb receives special attention because it is responsible for many precision grasps.

Instead of reproducing every biological thumb joint, the authors simplify the mechanism while preserving its functional capabilities.

The thumb mainly provides:

- Opposition
- Flexion
- Precision pinch
- Power grasp
- Tripod grasp

This significantly reduces mechanical complexity while maintaining high dexterity.

---

# 9. Physical Intelligence

Perhaps the biggest contribution of this paper is the concept of **Physical Intelligence**.

Traditional robotics often attempts to solve uncertainty through increasingly sophisticated software.

The ADAPT philosophy is different.

Instead of relying solely on sensing and computation, the mechanical structure itself contributes to solving the task.

Good mechanical design therefore reduces the complexity of control.

This idea appears repeatedly throughout the paper.

---

# 10. Open-Loop Grasping

One of the most surprising aspects of the ADAPT Hand is that it performs grasping using identical open-loop waypoint sequences.

The robot executes the same predefined motion regardless of whether it encounters an apple, a bottle, or several smaller objects.

The reason this works is not because of advanced software.

Instead, compliance allows the fingers to reorganize themselves after making contact.

The object itself influences the final finger configuration.

This phenomenon is referred to as **self-organization**.

---

# 11. Validation

The authors validate the hand using several experiments.

These include:

- Human grasp taxonomy
- Kapandji thumb evaluation
- Open-loop grasping
- Robustness against object displacement
- Grasping household objects

The experiments demonstrate that mechanical intelligence allows successful grasping despite uncertainty in object position.

---

# 12. ROS2 and micro-ROS

Although not the primary focus of this paper, the complete robotic system integrates the hand using ROS2 and micro-ROS.

Their roles can be understood at a high level as follows:

ROS2 is responsible for:

- High-level planning
- Motion coordination
- Communication with the robotic arm
- Supervisory control

micro-ROS executes on embedded microcontrollers and handles:

- Motor control
- Encoder feedback
- Low-level communication
- Real-time execution

The overall architecture can be summarized as:

ROS2

↓

micro-ROS

↓

Motor Controllers

↓

Tendons

↓

Finger Motion

---

# 13. Main Contributions

The most significant contributions of this work include:

- Anthropomorphic tendon-driven robotic hand
- Distributed compliance
- Underactuated tendon routing
- Physical intelligence
- Self-organizing grasps
- Robust open-loop manipulation
- Reduced dependence on sensing

---

# 14. Limitations

Despite its impressive design, the ADAPT Hand still has several limitations.

- Stiffness cannot be actively varied.
- Compliance remains fixed after fabrication.
- Large objects remain outside the grasp workspace.
- The thumb is functionally simplified compared to the biological thumb.
- Precision manipulation is still limited compared to fully actuated robotic hands.

---

# 15. Comparison with the Previous Paper (TUM Variable Stiffness Finger)

The previous paper focused on designing a single tendon-driven robotic finger capable of actively modifying its joint stiffness using Soft Pneumatic Elastomer Actuators (SPEAs).

Its primary contribution was demonstrating that finger stiffness could be controlled independently of tendon-driven motion. This enabled the same finger to remain compliant while approaching an object and become rigid once grasping was completed.

The ADAPT Hand addresses a different challenge.

Instead of designing a single finger, it focuses on constructing an entire anthropomorphic robotic hand.

Rather than actively changing stiffness, the authors introduce distributed compliance throughout the fingertips, fingers, and wrist. Their philosophy is that intelligent mechanical design can simplify control and naturally produce robust grasps.

Consequently, the two papers are complementary rather than competing.

The first teaches **how to control stiffness**, whereas the second teaches **how to design an intelligent compliant hand**.

---

# 16. Combining Both Papers

One possible future research direction would be to integrate the strengths of both papers.

A robotic hand could use:

- The tendon routing and anthropomorphic architecture proposed by ADAPT.

combined with

- The actively controllable variable stiffness joints introduced in the TUM paper.

Such a system could:

- Approach objects safely using low stiffness.
- Adapt naturally during contact.
- Increase stiffness after grasping.
- Improve payload capacity.
- Improve grasp stability.
- Handle both fragile and heavy objects using the same hand.

This combination appears to be a promising direction for future tendon-driven anthropomorphic robotic hands.

---

# 17. Ideas Useful for My Research

After studying this paper, the following engineering ideas appear particularly useful for my own tendon-driven hand project.

### Distributed Compliance

Compliance should exist not only at the finger joints but also at the fingertips and wrist.

### Underactuated Tendon Routing

Reducing actuator count while preserving dexterity is an important design objective.

### Remote Actuation

Keeping motors inside the palm reduces finger inertia and improves overall dynamics.

### Mechanical Intelligence

Rather than solving every problem through software, mechanical design should naturally encourage successful grasps.

### Open-Loop Manipulation

Good mechanics can significantly reduce the complexity of the control architecture.

---

# 18. Evolution of My Understanding

## Before Reading This Paper

- I believed that dexterous robotic hands required one actuator for every joint.
- I mainly associated compliance with soft robotic fingers.
- I thought successful grasping depended primarily on sophisticated control algorithms.

## After Reading This Paper

- I now understand that underactuation allows fewer actuators to generate complex grasp behaviors.
- Compliance can be distributed throughout the entire hand rather than existing only at the joints.
- Mechanical intelligence can significantly simplify robot control.
- Tendon routing is just as important as the controller itself.
- Intelligent mechanics often outperform unnecessarily complicated software.

---

# 19. Questions for Future Reading

While reading this paper, several interesting research questions emerged.

- Can actively variable stiffness be integrated into the ADAPT architecture?
- What is the optimal tendon routing strategy for maximum dexterity?
- How should tendon tension be regulated during grasping?
- Can reinforcement learning exploit passive compliance?
- How can tactile sensing further improve grasp robustness?
- How should the thumb mechanism be optimized for precision manipulation?

---

# 20. Personal Reflection

This paper fundamentally changed my perspective on robotic hand design.

Initially, I believed that improving robotic manipulation primarily required increasingly sophisticated controllers and sensing systems.

The ADAPT Hand demonstrates a different philosophy.

By carefully designing the mechanical structure—through tendon routing, underactuation, and distributed compliance—the complexity of control can be significantly reduced while simultaneously improving robustness.

The most important lesson I take from this work is that good robotic systems are not merely software-driven; they emerge from a harmonious balance between mechanical design, actuation, compliance, and control.

As I continue developing a tendon-driven anthropomorphic robotic hand, this paper will serve as an important reference for understanding how intelligent mechanical design can naturally produce adaptable and robust grasping behavior.