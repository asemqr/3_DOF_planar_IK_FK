Trajectory Generation and Kinematics for a 3-DOF Planar Manipulator

Index Terms — ROS, Inverse Kinematics, Forward Kinematics, 3 DOF planar manipulator

  I. Introduction

In this lab, we explore the implementation of trajectory generation for robotic joint movements using cubic polynomials. The focus is on trajectory planning within the robot’s workspace — both without and with via points — and performing forward and inverse kinematics calculations for the 3-DOF system. These are essential for determining both the joint angles needed to achieve specific end-effector positions and the position of the end-effector based on given joint angles.

For the real robot, we used a 5-degree-of-freedom (5-DOF) robotic arm, deactivating joints 2 and 4 to obtain a simplified 3-DOF configuration. For the simulation, we recreated the robot in the Gazebo environment using the dimensions of the real robot.

Through these exercises, we gain a deeper understanding of trajectory planning and control, ensuring the robot can execute smooth movements across its workspace.

---

  II. Methodology

   A. Task 1

   1. Calculating Forward Kinematics

Forward kinematics is used to determine the position and orientation of the end-effector given the joint parameters. We derive forward kinematics using the Denavit-Hartenberg (DH) convention for a 3-DOF robot.

- We assume that the zero frame and frame 1 are placed on the same axis at the first joint, making `a₀ = 0`.

   2. Calculating the DH Parameters

Due to our assumption:

- `a₀ = 0` (coincident frames).
- `a₁`, `a₂` are the physical lengths of the links.

DH Table:

| Link i        | αᵢ₋₁ (deg) | aᵢ₋₁ (mm) | dᵢ (mm) | θᵢ (deg) |
|---------------|------------|------------|----------|-----------|
| 1             | 0          | 0          | 0        | θ₁        |
| 2             | 0          | 145        | 0        | θ₂        |
| 3             | 0          | 145        | 0        | θ₃        |
| End-effector  | 0          | 45         | N/A      | N/A       |

Parameter Definitions:

- αᵢ₋₁: Twist angle — angle between zᵢ₋₁ and zᵢ about xᵢ₋₁.
- aᵢ₋₁: Link length — distance between zᵢ₋₁ and zᵢ along xᵢ₋₁.
- dᵢ: Link offset — distance along zᵢ₋₁ between frames.
- θᵢ: Joint angle — angle between xᵢ₋₁ and xᵢ about zᵢ₋₁.

---

   B. Forward Kinematics Derivation Using DH Parameters

Using the standard Denavit-Hartenberg transformation matrix:

```
Tᵢ₋₁ⁱ = [
  [cosθᵢ, -sinθᵢcosαᵢ₋₁, sinθᵢsinαᵢ₋₁, aᵢ₋₁cosθᵢ],
  [sinθᵢ, cosθᵢcosαᵢ₋₁, -cosθᵢsinαᵢ₋₁, aᵢ₋₁sinθᵢ],
  [0, sinαᵢ₋₁, cosαᵢ₋₁, dᵢ],
  [0, 0, 0, 1]
]
```

   Transformation Matrices

From Base to Link 1:

```
T₀₁ = [
  [cosθ₁, -sinθ₁, 0, 0],
  [sinθ₁, cosθ₁, 0, 0],
  [0, 0, 1, 0],
  [0, 0, 0, 1]
]
```

From Link 1 to Link 2:

```
T₁₂ = [
  [cosθ₂, -sinθ₂, 0, 145cosθ₂],
  [sinθ₂, cosθ₂, 0, 145sinθ₂],
  [0, 0, 1, 0],
  [0, 0, 0, 1]
]
```

From Link 2 to Link 3:

```
T₂₃ = [
  [cosθ₃, -sinθ₃, 0, 145cosθ₃],
  [sinθ₃, cosθ₃, 0, 145sinθ₃],
  [0, 0, 1, 0],
  [0, 0, 0, 1]
]
```

From Link 3 to End-Effector:

```
T₃_EE = [
  [1, 0, 0, 45],
  [0, 1, 0, 0],
  [0, 0, 1, 0],
  [0, 0, 0, 1]
]
```

   Total Transformation Matrix

```
T_total = T₀₁ · T₁₂ · T₂₃ · T₃_EE
```

Final matrix provides end-effector position and orientation in base frame.

---

   C. Kinematic Modeling and Workspace Visualization

- Implemented `dh_transform()` for computing transformation matrices.
- `forward_kinematics()` calculates end-effector position from joint angles.
- `plot_workspace()` iterates through joint angle ranges, computes reachable positions, and plots them in 2D.

Link lengths used:
- L1 = 135 mm
- L2 = 135 mm
- L3 = 46.7 mm

Figures:
- Fig. 1: Workspace using ±90° joint range
- Fig. 2: Workspace using ±45° joint range

---

   D. Task 2

Implemented Functions:

- `cubic_coefficients`: Calculates coefficients for smooth motion between initial and final joint positions.
- `cubic_trajectory`: Evaluates trajectory at any time using those coefficients.

Joint Movements:

- Joint 1: 0 → -0.8 radians
- Joint 2: 0 → 0.8 radians
- Joint 3: 1.57 → 0.8 radians

Output:

- Generated smooth trajectories over 5 seconds.
- Plots confirm correct cubic trajectory generation.
Task 3: Inverse Kinematics with Orientation
In this task, we implemented the function inverse_kinematics_with_orientation for a 3-DOF planar manipulator. The goal was to compute the joint angles (q1, q2, q3) given the desired end-effector position and orientation. Due to an end-effector offset of L3 = 45 mm, we first determined the wrist position before applying inverse kinematics.

1. Position of the End-Effector
Given:

End-effector position (x, y)

Orientation ϕ (relative to x-axis)

Offset L3

We calculate the wrist position as:

xwrist = x - L3 * cos(ϕ)
ywrist = y - L3 * sin(ϕ)

2. Inverse Kinematics Equations
Using wrist coordinates, we apply the inverse kinematics:

xwrist = l1 * cos(θ1) + l2 * cos(θ1 + θ2)
ywrist = l1 * sin(θ1) + l2 * sin(θ1 + θ2)

Squaring and adding:

xwrist² + ywrist² = l1² + l2² + 2 * l1 * l2 * cos(θ2)

Solving for cos(θ2):

cos(θ2) = (xwrist² + ywrist² - l1² - l2²) / (2 * l1 * l2)

3. Solving for θ2
θ2 = atan2(±sqrt(1 - cos²(θ2)), cos(θ2))

Choose positive or negative square root for elbow-up or elbow-down configuration.

4. Solving for θ1
Let:

k1 = l2 * cos(θ1 + θ2)

k2 = l2 * sin(θ1 + θ2)

θ1 = atan2(ywrist, xwrist) - atan2(k2, k1)

5. Solving for θ3
θ3 = ϕ - (θ1 + θ2)

6. Summary of Equations
xwrist = x - L3 * cos(ϕ)

ywrist = y - L3 * sin(ϕ)

cos(θ2) = (xwrist² + ywrist² - l1² - l2²) / (2 * l1 * l2)

θ2 = atan2(±sqrt(1 - cos²(θ2)), cos(θ2))

θ1 = atan2(ywrist, xwrist) - atan2(k2, k1)

θ3 = ϕ - (θ1 + θ2)

Example
The program allows the user to select a point in the workspace. It calculates the joint angles if the point is within reach.

Implementation Overview
Denavit-Hartenberg Transformation
Implemented dh_transform to compute transformation matrices using theta, d, a, and alpha parameters.

Forward Kinematics
Function calculates the end-effector position from θ1, θ2, θ3 using the D-H convention.

Inverse Kinematics
Given (x, y, ϕ), it returns θ1, θ2, and θ3. If the point is unreachable, an error message is displayed.

Plotting Workspace
Simulates joint angles across full ranges and plots reachable positions.

User Interaction
Mouse click handler allows users to select a target. If reachable, it shows the joint angles needed.

Example Application
Link lengths used: L1 = 135 mm, L2 = 135 mm, L3 = 46.7 mm. Program visualizes reachable area and allows interaction.

Gazebo Simulation
The manipulator moves to selected positions in Gazebo. The same logic is tested on a real robot.

Task 4: Configuration Visualization
We implemented the function plot_manipulator to visualize the manipulator's joint positions and end-effector orientation.

D-H transformations used to compute positions

Cubic polynomial trajectory created for smooth transition

Initial and final configurations clearly marked

plot_trajectory_within_workspace overlays trajectory on workspace

Start point marked with red, end point with green

This function provides clear visualization of both configurations and movement.

Task 5: Trajectories Through Via Points
Implemented joint-space cubic trajectories through multiple via points.

Uses inverse kinematics and cubic polynomial interpolation

Handles up to 5 via points for smoother movement

Unreachable positions are flagged

Function plot_via_points_trajectory:

Inputs: via points, time interval, boundary conditions

Generates smooth joint angle transitions

Plots end-effector trajectory and joint angles

Tested on Gazebo and real robot. Shows movement from start to goal via intermediate points.
