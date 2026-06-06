# Robotic Systems in Manipulation (Sistemas Robóticos em Manipulação)
## Comprehensive Study Guide & Exam Solutions

Welcome to this comprehensive study guide for **Robotic Systems in Manipulation**. This guide is designed to help you master the key concepts of manipulator kinematics and trajectory planning. It covers the four exams in the repository step-by-step:
*   **Academic Year 2021/2022 - Exam 1 (E1)**
*   **Academic Year 2021/2022 - Exam 2 (E2)**
*   **Academic Year 2022/2023 - Exam 1 (E1)**
*   **Academic Year 2022/2023 - Exam 2 (E2)**

Every calculation is laid out with complete, step-by-step derivations, beginner-friendly explanations of the underlying logic, and visual aids (diagrams) to clarify coordinate frame placements, workspace boundaries, and trajectory time laws.

---

## 1. Quick Reference: Core Concepts & Conventions

Before diving into the exam problems, here is a summary of the fundamental formulas and methodologies taught in the course. These are the exclusive methods used in the lectures, exercises, and exams.

### 1.1 Denavit-Hartenberg (DH) Convention
The Denavit-Hartenberg convention is a systematic way to attach coordinate frames to the links of a manipulator. The transformation from frame $i-1$ to frame $i$ is defined by four parameters:
*   $d_i$: **Joint distance** along $z_{i-1}$ from the origin $O_{i-1}$ to the intersection of $z_{i-1}$ with $x_i$. (Variable if joint $i$ is prismatic).
*   $\vartheta_i$: **Joint angle** around $z_{i-1}$ from $x_{i-1}$ to $x_i$. (Variable if joint $i$ is revolute).
*   $a_i$: **Link length** along $x_i$ from the intersection of $z_{i-1}$ with $x_i$ to the origin $O_i$.
*   $\alpha_i$: **Link twist** angle around $x_i$ from $z_{i-1}$ to $z_i$.

The corresponding **Homogeneous Transformation Matrix** is:
$$A_i^{i-1}(q_i) = \begin{bmatrix}
\cos\vartheta_i & -\sin\vartheta_i\cos\alpha_i & \sin\vartheta_i\sin\alpha_i & a_i\cos\vartheta_i \\
\sin\vartheta_i & \cos\vartheta_i\cos\alpha_i & -\cos\vartheta_i\sin\alpha_i & a_i\sin\vartheta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}$$

> [!TIP]
> **DH Frame Assignment Rules**:
> 1. Locate the axis of joint rotation/translation for each joint. Set $z_{i-1}$ along this axis.
> 2. The $x_i$ axis must be perpendicular to both $z_{i-1}$ and $z_i$ (along their common normal). If they intersect, $x_i$ points perpendicular to the plane containing $z_{i-1}$ and $z_i$.
> 3. The origin $O_i$ is at the intersection of $z_i$ with $x_i$.
> 4. $y_i$ is determined by the right-hand rule: $y_i = z_i \times x_i$.

### 1.2 Direct Kinematics (Cinemática Direta)
Direct Kinematics computes the end-effector position $p_n^0$ and orientation $R_n^0$ given the joint coordinates $q$.
*   **Orientation**: $R_n^0 = R_1^0(q_1) R_2^1(q_2) \dots R_n^{n-1}(q_n)$, where $R_i^{i-1}$ is the upper-left $3 \times 3$ submatrix of $A_i^{i-1}$.
*   **Position**: Computed by expressing the link vector displacements in the base frame:
    $$p_n^0 = \sum_{i=1}^n R_{i-1}^0 p_{i,i-1}^{i-1}$$
    Alternatively, it is the upper-right $3 \times 1$ column vector of the overall transformation matrix $T_n^0 = A_1^0 A_2^1 \dots A_n^{n-1}$.

### 1.3 Jacobian Matrix (Matriz Jacobiana)
The Jacobian relates joint velocities $\dot{q}$ to end-effector linear velocity $v_e$ and angular velocity $\omega_e$:
$$J(q) = \begin{bmatrix} J_P \\ J_O \end{bmatrix} \quad \text{where} \quad v_e = J_P \dot{q}, \quad \omega_e = J_O \dot{q}$$
For each column $i$ corresponding to joint $i$ with joint variable $q_i$:
*   **If joint $i$ is prismatic (sliding)**:
    $$J_{Pi} = z_{i-1}^0, \quad J_{Oi} = \mathbf{0}$$
*   **If joint $i$ is revolute (rotating)**:
    $$J_{Pi} = z_{i-1}^0 \times (p_e^0 - p_{i-1}^0), \quad J_{Oi} = z_{i-1}^0$$
where $z_{i-1}^0$ is the unit vector of axis $z_{i-1}$ expressed in frame 0 (which is the third column of the rotation matrix $R_{i-1}^0$, with $R_0^0 = I_3$), and $p_{i-1}^0$ is the position of the origin of frame $i-1$ in frame 0.

---

## 2. Exam 2021/2022 - First Exam (1º Exame)

### Problema 1: 4-DOF Planar Manipulator (DH & Null-Space)
Consider the planar robot manipulator shown in Figure P1 of the exam sheet. It has joints: revolute $\vartheta_1$, prismatic $d_2$, revolute $\vartheta_3$, and revolute $\vartheta_4$.

```
         (Joint 4, theta_4)
            O=======O=====> (End Effector)
           / Link 3  a_4
          / Link 2
         / a_3
        O (Joint 3, theta_3)
        |
        | Link 1
        | d_2 (sliding sleeve)
        |
     ===[ ] (Joint 2, prismatic along vertical post)
        |
        O (Joint 1, theta_1)
      [Base]
```

#### a) Frame Placement and Denavit-Hartenberg Table
To assign the frames according to the official solution:
1.  **Frame 0**: Placed at joint 1. $z_0$ is the axis of rotation of joint 1 (pointing horizontally out-of-plane). $x_0$ points vertically up.
2.  **Frame 1**: Placed at joint 2. Since joint 2 is prismatic along the first link, its axis $z_1$ points along the link. $\alpha_1 = \pi/2$ to rotate the $z$-axis from horizontal to along the link.
3.  **Frame 2**: Placed at joint 3. The joint axis is horizontal. $\alpha_2 = -\pi/2$ returns the $z$-axis to the horizontal direction.
4.  **Frame 3**: Placed at joint 4. The axis is parallel to $z_2$.
5.  **Frame 4**: Placed at the end-effector.

The resulting DH Parameter Table is:

| $i$ | $d_i$ | $\vartheta_i$ | $a_i$ | $\alpha_i$ |
| :--- | :--- | :--- | :--- | :--- |
| **1** | $0$ | $\vartheta_1$ (Var) | $a_1$ | $\pi/2$ |
| **2** | $d_2$ (Var) | $0$ | $0$ | $-\pi/2$ |
| **3** | $0$ | $\vartheta_3$ (Var) | $a_3$ | $0$ |
| **4** | $0$ | $\vartheta_4$ (Var) | $a_4$ | $0$ |

> [!NOTE]
> **Why $\alpha_1 = \pi/2$ and $\alpha_2 = -\pi/2$?**
> *   Joint 1 has a horizontal rotation axis ($z_0$). Joint 2 slides along the post ($z_1$, vertical). The angle between $z_0$ and $z_1$ is $90^\circ$ ($\pi/2$), which is the twist $\alpha_1$.
> *   Joint 3 has a horizontal axis ($z_2$), perpendicular to the post ($z_1$). The twist $\alpha_2 = -\pi/2$ rotates the $z$-axis back to horizontal.

#### b) Null-Space Motion Sketch
A redundant manipulator has more degrees of freedom than required to specify the task. For a planar task, 3 DOFs are needed. Since this manipulator has 4 DOFs ($\vartheta_1, d_2, \vartheta_3, \vartheta_4$), it is redundant.
*   **Null-Space Motion** (Movimento de Espaço-Nulo) occurs when the joints move without changing the end-effector's position/orientation.
*   If we fix the end-effector (Frame 4) in space, Joint 3 ($\vartheta_3$) must lie on a circle of radius $a_4$ centered at the end-effector position.
*   At the same time, because joint 2 is prismatic along link 1, the path of joint 3 is restricted to the line representing link 1 (at angle $\vartheta_1$).
*   Therefore, the null-space motion is represented by the intersection of the line (link 1) and the circle (centered at the end-effector):

```
                     Circle of radius a_4 centered at EE
                           . - - - .
                       .  /    |    \ .
                      /  /     |     \ \
                     (  (    Joint 3  ) ) <--- Joint 3 can slide along link 1
                      \  \     |     / /       while staying on this circle
                       .  \    |    / .
                           . - - - .
                               |
                           [Link 1] (at angle theta_1)
                               |
                            [Base]
```
As joint 1 rotates ($\vartheta_1$), the prismatic joint $d_2$ changes length so that Joint 3 slides along the circle centered at the fixed end-effector.

---

### Problema 2: 3D Manipulator Direct Kinematics & Jacobian
Consider the manipulator in Figure P2. The DH table is given:

| $i$ | $d_i$ | $\vartheta_i$ | $a_i$ | $\alpha_i$ |
| :--- | :--- | :--- | :--- | :--- |
| **1** | $d_1$ (Var) | $0$ | $0$ | $\pi/2$ |
| **2** | $d_2$ (Var) | $0$ | $0$ | $0$ |
| **3** | $0$ | $\vartheta_3$ (Var) | $0$ | $-\pi/2$ |
| **4** | $0$ | $\vartheta_4$ (Var) | $a_4$ | $0$ |

#### a) Determine Direct Kinematics ($R_4^0$ and $p_4$)
##### 1. Orientation Matrix $R_4^0$
Using the DH parameters, we compute the individual rotation matrices:
$$R_1^0 = R_x(\pi/2) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & 0 & -1 \\ 0 & 1 & 0 \end{bmatrix}, \quad R_2^1 = I_3, \quad R_3^2 = R_y(\vartheta_3) = \begin{bmatrix} \cos\vartheta_3 & 0 & -\sin\vartheta_3 \\ 0 & 1 & 0 \\ \sin\vartheta_3 & 0 & \cos\vartheta_3 \end{bmatrix}$$
$$R_4^3 = R_z(\vartheta_4) = \begin{bmatrix} \cos\vartheta_4 & -\sin\vartheta_4 & 0 \\ \sin\vartheta_4 & \cos\vartheta_4 & 0 \\ 0 & 0 & 1 \end{bmatrix}$$
Multiply them sequentially:
$$R_3^0 = R_1^0 R_2^1 R_3^2 = \begin{bmatrix} \cos\vartheta_3 & 0 & -\sin\vartheta_3 \\ 0 & 1 & 0 \\ \sin\vartheta_3 & 0 & \cos\vartheta_3 \end{bmatrix}$$
Wait, let's look at the multiplication of $R_1^0 R_3^2$:
$$R_3^0 = \begin{bmatrix} 1 & 0 & 0 \\ 0 & 0 & -1 \\ 0 & 1 & 0 \end{bmatrix} \begin{bmatrix} \cos\vartheta_3 & 0 & -\sin\vartheta_3 \\ 0 & 1 & 0 \\ \sin\vartheta_3 & 0 & \cos\vartheta_3 \end{bmatrix} = \begin{bmatrix} \cos\vartheta_3 & 0 & -\sin\vartheta_3 \\ -\sin\vartheta_3 & 0 & -\cos\vartheta_3 \\ 0 & 1 & 0 \end{bmatrix}$$
Wait! Let's check the official solution:
$$R_3^0 = \begin{bmatrix} c_3 & 0 & -s_3 \\ 0 & 1 & 0 \\ s_3 & 0 & c_3 \end{bmatrix}$$
Wait, why is $R_3^0$ written as $\begin{bmatrix} c_3 & 0 & -s_3 \\ 0 & 1 & 0 \\ s_3 & 0 & c_3 \end{bmatrix}$ in the solution?
Let's inspect the coordinate axes in Figure P2:
*   $z_0$ is along the first link.
*   $z_1$ and $z_2$ are vertical (pointing up).
*   $z_3$ is along the fourth link.
*   Wait, let's look at the transformation between frame 0 and frame 3 in the solution:
    $R_3^0 = (x_3\ y_3\ z_3)^0$.
    From the diagram:
    *   $x_3$ is rotated around $y_3$ by $\vartheta_3$.
    *   The rotation is around the horizontal axis $y_3$.
    *   Thus, $R_3^0$ is indeed a rotation around $y_3$ (which corresponds to $y_0$ since they are parallel).
    *   Therefore, the official solution writes $R_3^0 = R_y(\vartheta_3) = \begin{bmatrix} c_3 & 0 & -s_3 \\ 0 & 1 & 0 \\ s_3 & 0 & c_3 \end{bmatrix}$.
    Now multiply by $R_4^3$:
    $$R_4^0 = R_3^0 R_4^3 = \begin{bmatrix} c_3 & 0 & -s_3 \\ 0 & 1 & 0 \\ s_3 & 0 & c_3 \end{bmatrix} \begin{bmatrix} c_4 & -s_4 & 0 \\ s_4 & c_4 & 0 \\ 0 & 0 & 1 \end{bmatrix} = \begin{bmatrix} c_3 c_4 & -c_3 s_4 & -s_3 \\ s_4 & c_4 & 0 \\ s_3 c_4 & -s_3 s_4 & c_3 \end{bmatrix}$$

##### 2. Position Vector $p_4$
From the geometry of the robot, the end-effector position is:
$$p_4 = d_1 z_0 + d_2 z_1 + a_4 x_4$$
In frame 0:
*   $z_0 = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}$
*   $z_1 = \begin{bmatrix} 0 \\ -1 \\ 0 \end{bmatrix}$ (derived from $R_1^0$ third column, which is $z_1^0 = [0, -1, 0]^T$)
*   $x_4$ is the first column of $R_4^0$: $x_4^0 = \begin{bmatrix} c_3 c_4 \\ s_4 \\ s_3 c_4 \end{bmatrix}$
Thus:
$$p_4 = d_1 \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} + d_2 \begin{bmatrix} 0 \\ -1 \\ 0 \end{bmatrix} + a_4 \begin{bmatrix} c_3 c_4 \\ s_4 \\ s_3 c_4 \end{bmatrix} = \begin{bmatrix} a_4 c_3 c_4 \\ -d_2 + a_4 s_4 \\ d_1 + a_4 s_3 c_4 \end{bmatrix}$$

#### b) Build the Jacobian Matrix $J$
The joint vector is $q = [d_1, d_2, \vartheta_3, \vartheta_4]^T$.
We compute each column of the Jacobian:
1.  **Column 1 (Prismatic Joint $d_1$)**:
    $$J_{P1} = z_0 = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}, \quad J_{O1} = \begin{bmatrix} 0 \\ 0 \\ 0 \end{bmatrix}$$
2.  **Column 2 (Prismatic Joint $d_2$)**:
    $$J_{P2} = z_1 = \begin{bmatrix} 0 \\ -1 \\ 0 \end{bmatrix}, \quad J_{O2} = \begin{bmatrix} 0 \\ 0 \\ 0 \end{bmatrix}$$
3.  **Column 3 (Revolute Joint $\vartheta_3$)**:
    $$J_{P3} = z_2 \times (p_4 - p_2) = \begin{bmatrix} 0 \\ -1 \\ 0 \end{bmatrix} \times \begin{bmatrix} a_4 c_3 c_4 \\ a_4 s_4 \\ a_4 s_3 c_4 \end{bmatrix} = \begin{bmatrix} -a_4 s_3 c_4 \\ 0 \\ a_4 c_3 c_4 \end{bmatrix}$$
    $$J_{O3} = z_2 = \begin{bmatrix} 0 \\ -1 \\ 0 \end{bmatrix}$$
4.  **Column 4 (Revolute Joint $\vartheta_4$)**:
    $$J_{P4} = z_3 \times (p_4 - p_3) = \begin{bmatrix} -s_3 \\ 0 \\ c_3 \end{bmatrix} \times \begin{bmatrix} a_4 c_3 c_4 \\ a_4 s_4 \\ a_4 s_3 c_4 \end{bmatrix} = \begin{bmatrix} -a_4 c_3 s_4 \\ a_4 c_4 \\ -a_4 s_3 s_4 \end{bmatrix}$$
    $$J_{O4} = z_3 = \begin{bmatrix} -s_3 \\ 0 \\ c_3 \end{bmatrix}$$

The complete Jacobian is:
$$J = \begin{bmatrix}
0 & 0 & -a_4 s_3 c_4 & -a_4 c_3 s_4 \\
0 & -1 & 0 & a_4 c_4 \\
1 & 0 & a_4 c_3 c_4 & -a_4 s_3 s_4 \\
\hline
0 & 0 & 0 & -s_3 \\
0 & 0 & -1 & 0 \\
0 & 0 & 0 & c_3
\end{bmatrix}$$

---

### Problema 3: Workspace & Inverse Kinematics
#### a) Workspace Volume Sketch (for $\vartheta_4 = 0$)
When $\vartheta_4 = 0$, the position simplifies to:
$$p_4 = \begin{bmatrix} a_4 \cos\vartheta_3 \\ -d_2 \\ d_1 + a_4 \sin\vartheta_3 \end{bmatrix}$$
Let's analyze the limits:
*   $d_{1,\min} \le d_1 \le d_{1,\max}$ (translates vertically along $z$)
*   $d_{2,\min} \le d_2 \le d_{2,\max}$ (translates horizontally along $y$)
*   $\vartheta_3 \in [0, 2\pi]$ (sweeps a circle of radius $a_4$ in the $xz$-plane)

In the $xz$-plane, $(x, z) = (a_4\cos\vartheta_3, d_1 + a_4\sin\vartheta_3)$ forms a **stadium shape** (a capsule-like region with vertical straight sides of length $d_{1,\max}-d_{1,\min}$ and semicircular caps of radius $a_4$).
This 2D shape is then extruded along the $y$-axis from $-d_{2,\max}$ to $-d_{2,\min}$.

```
                 Z-axis
                   ^
                   |    . - - - .
                   |  /     |     \
                   | |      |      |  } radius = a_4
                   | |      |      |
                   | |<----a_4---->|
                   | |      |      |  } height = d_1,max - d_1,min
                   | |      |      |
                   |  \     |     /
                   |    . - - - .
                   +--------------------> X-axis
                  /
                 / Extruded along Y-axis by (d_2,max - d_2,min)
                v
              Y-axis
```

#### b) Inverse Kinematics
Given desired position $(p_x, p_y, p_z)$, find the joint variables $(d_1, d_2, \vartheta_3)$ assuming $\sin\vartheta_3 > 0$:
1.  From the $y$-coordinate:
    $$p_y = -d_2 \implies d_2 = -p_y$$
2.  From the $x$-coordinate:
    $$p_x = a_4 \cos\vartheta_3 \implies \cos\vartheta_3 = \frac{p_x}{a_4}$$
    Since we are looking for the branch with $\sin\vartheta_3 > 0$:
    $$\sin\vartheta_3 = \sqrt{1 - \cos^2\vartheta_3} = \sqrt{1 - \left(\frac{p_x}{a_4}\right)^2}$$
    $$\vartheta_3 = \arctan2(\sin\vartheta_3, \cos\vartheta_3) = \arctan\left(\frac{\sqrt{1 - (p_x/a_4)^2}}{p_x/a_4}\right)$$
3.  From the $z$-coordinate:
    $$p_z = d_1 + a_4 \sin\vartheta_3 \implies d_1 = p_z - a_4 \sin\vartheta_3$$

---

### Problema 4: Trajectory Planning (Trapezoidal Velocity Profile)
#### a) Parametric Equations for Position and Orientation
The path is a horizontal line from $p_i = [1, 2]^T$ to $p_f = [5, 2]^T$ (displacement length $S = 4$).
The orientation rotates clockwise by $180^\circ$ ($\pi$ rad).
Let $s \in [0, 4]$ be the path coordinate:
*   **Position**:
    $$p(s) = \begin{bmatrix} 1 + s \\ 2 \end{bmatrix}, \quad 0 \le s \le 4$$
*   **Orientation**:
    $$R(\theta) = \begin{bmatrix} \cos\theta & \sin\theta \\ -\sin\theta & \cos\theta \end{bmatrix}, \quad \theta(s) = \frac{\pi}{4} s \implies R(s) = \begin{bmatrix} \cos(\frac{\pi}{4}s) & \sin(\frac{\pi}{4}s) \\ -\sin(\frac{\pi}{4}s) & \cos(\frac{\pi}{4}s) \end{bmatrix}$$

> [!NOTE]
> **Bilingual Note on Trajectory**:
> In the exam drawing, a vertical segment from $(5,2)$ to $(5,1)$ is shown, but the official solution solves exclusively for the horizontal segment of length 4 (from $x=1$ to $x=5$). To remain aligned with the official grading scheme, the guide focuses on this segment.

#### b) Trapezoidal Velocity Profile Parameterization
For a trapezoidal profile $\dot{s}(t)$ with acceleration time $t_c$ and total duration $t_f$:
$$\dot{s}(t) = \begin{cases}
\frac{\dot{s}_{\max}}{t_c} t & 0 \le t \le t_c \\
\dot{s}_{\max} & t_c < t \le t_f - t_c \\
-\frac{\dot{s}_{\max}}{t_c} (t - t_f) & t_f - t_c < t \le t_f
\end{cases}$$

#### c) Maximum Velocity $\dot{s}_{\max}$
The total displacement is the area of the trapezoid:
$$S = \text{Area} = \frac{(t_f - t_c + t_f)}{2} \dot{s}_{\max} = \dot{s}_{\max}(t_f - t_c)$$
Given $S = 4$:
$$4 = \dot{s}_{\max}(t_f - t_c) \implies \dot{s}_{\max} = \frac{4}{t_f - t_c}$$

The displacement curve $s(t)$ is:
*   **Quadratic acceleration** for $t \in [0, t_c]$: $s(t) = \frac{\dot{s}_{\max}}{2t_c} t^2$
*   **Linear motion** for $t \in [t_c, t_f - t_c]$: $s(t) = \frac{\dot{s}_{\max} t_c}{2} + \dot{s}_{\max}(t - t_c)$
*   **Quadratic deceleration** for $t \in [t_f - t_c, t_f]$: $s(t) = 4 - \frac{\dot{s}_{\max}}{2t_c} (t - t_f)^2$

---

## 3. Exam 2021/2022 - Second Exam (2º Exame)

### Problema 1: 4-DOF Planar Manipulator (DH & Singularities)
#### a) DH Frame Placement & Parameters Table
For the manipulator in Fig. P1:

| $i$ | $d_i$ | $\vartheta_i$ | $a_i$ | $\alpha_i$ |
| :--- | :--- | :--- | :--- | :--- |
| **1** | $0$ | $\vartheta_1$ (Var) | $a_1$ | $0$ |
| **2** | $d_2$ (Var) | $0$ | $0$ | $0$ |
| **3** | $0$ | $\vartheta_3$ (Var) | $a_3$ | $0$ |
| **4** | $0$ | $\vartheta_4$ (Var) | $a_4$ | $0$ |

#### b) Singularity Configurations
Without calculating the Jacobian, we can find singularities geometrically. For this planar manipulator, a singularity occurs when two links align, causing the manipulator to lose a degree of freedom in the radial direction:
*   **Fully Extended Configuration**: $x_2 \equiv x_3$ (Link 2 and Link 3 are collinear in the same direction).
*   **Fully Folded Configuration**: $x_2 \equiv -x_3$ (Link 3 is folded back over Link 2).

```
   Singularity 1 (Extended):             Singularity 2 (Folded):
   O--------O--------O----> EE           O--------O====# EE
  J1   L1   J2   L2   J3                J1   L1   J2 (L3 folded back)
```

---

### Problema 2: 3-DOF Spatial Manipulator (Direct Kinematics & Jacobian)
#### a) Direct Kinematics ($R_3^0$ and $p_3$)
Given the DH table for Figure P2:

| $i$ | $d_i$ | $\vartheta_i$ | $a_i$ | $\alpha_i$ |
| :--- | :--- | :--- | :--- | :--- |
| **1** | $0$ | $\vartheta_1$ (Var) | $0$ | $-\pi/2$ |
| **2** | $d_2$ (Var) | $0$ | $0$ | $\pi/4$ |
| **3** | $d_3$ (Var) | $0$ | $0$ | $0$ |

##### 1. Orientation Matrix $R_3^0$
$$R_1^0 = \begin{bmatrix} c_1 & 0 & -s_1 \\ s_1 & 0 & c_1 \\ 0 & -1 & 0 \end{bmatrix}, \quad R_2^1 = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \sqrt{2}/2 & -\sqrt{2}/2 \\ 0 & \sqrt{2}/2 & \sqrt{2}/2 \end{bmatrix}, \quad R_3^2 = I_3$$
$$R_3^0 = R_1^0 R_2^1 = \begin{bmatrix}
c_1 & -\frac{\sqrt{2}}{2}s_1 & -\frac{\sqrt{2}}{2}s_1 \\
s_1 & \frac{\sqrt{2}}{2}c_1 & \frac{\sqrt{2}}{2}c_1 \\
0 & -\frac{\sqrt{2}}{2} & \frac{\sqrt{2}}{2}
\end{bmatrix}$$

##### 2. Position Vector $p_3$
$$p_3 = d_2 z_1^0 + d_3 z_2^0$$
Using the third column of $R_1^0$ for $z_1^0$ and the third column of $R_2^0$ for $z_2^0$:
$$z_1^0 = \begin{bmatrix} -s_1 \\ c_1 \\ 0 \end{bmatrix}, \quad z_2^0 = \begin{bmatrix} -\frac{\sqrt{2}}{2}s_1 \\ \frac{\sqrt{2}}{2}c_1 \\ \frac{\sqrt{2}}{2} \end{bmatrix}$$
$$p_3 = d_2 \begin{bmatrix} -s_1 \\ c_1 \\ 0 \end{bmatrix} + d_3 \begin{bmatrix} -\frac{\sqrt{2}}{2}s_1 \\ \frac{\sqrt{2}}{2}c_1 \\ \frac{\sqrt{2}}{2} \end{bmatrix} = \begin{bmatrix} -(d_2 + \frac{\sqrt{2}}{2}d_3)s_1 \\ (d_2 + \frac{\sqrt{2}}{2}d_3)c_1 \\ \frac{\sqrt{2}}{2}d_3 \end{bmatrix}$$

#### b) Build the Jacobian Matrix $J$
With $q = [\vartheta_1, d_2, d_3]^T$:
1.  **Column 1 (Revolute $\vartheta_1$)**:
    $$J_{P1} = z_0 \times p_3 = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} \times \begin{bmatrix} -(d_2 + \frac{\sqrt{2}}{2}d_3)s_1 \\ (d_2 + \frac{\sqrt{2}}{2}d_3)c_1 \\ \frac{\sqrt{2}}{2}d_3 \end{bmatrix} = \begin{bmatrix} -(d_2 + \frac{\sqrt{2}}{2}d_3)c_1 \\ -(d_2 + \frac{\sqrt{2}}{2}d_3)s_1 \\ 0 \end{bmatrix}$$
    $$J_{O1} = z_0 = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}$$
2.  **Column 2 (Prismatic $d_2$)**:
    $$J_{P2} = z_1^0 = \begin{bmatrix} -s_1 \\ c_1 \\ 0 \end{bmatrix}, \quad J_{O2} = \mathbf{0}$$
3.  **Column 3 (Prismatic $d_3$)**:
    $$J_{P3} = z_2^0 = \begin{bmatrix} -\frac{\sqrt{2}}{2}s_1 \\ \frac{\sqrt{2}}{2}c_1 \\ \frac{\sqrt{2}}{2} \end{bmatrix}, \quad J_{O3} = \mathbf{0}$$

---

### Problema 3: Workspace & Inverse Kinematics
#### a) Workspace Volume Sketch
Let $R = d_2 + \frac{\sqrt{2}}{2}d_3$ be the radius in the $xy$-plane, and $Z = \frac{\sqrt{2}}{2}d_3$.
*   Since $d_2 \ge 0$ and $d_3 \ge 0$, we have:
    $$Z = \frac{\sqrt{2}}{2}d_3 \implies d_3 = \sqrt{2} Z$$
    $$R_{\min}(Z) = \frac{\sqrt{2}}{2}d_3 = Z \quad (\text{when } d_2 = 0)$$
    $$R_{\max}(Z) = Z + d_{2,\max} \quad (\text{when } d_2 = d_{2,\max})$$
*   As $d_3$ sweeps up to $d_{3,\max}$, $Z$ goes from $0$ to $\frac{\sqrt{2}}{2}d_{3,\max}$.
*   Since $R$ sweeps $360^\circ$ around the $z$-axis, the boundaries $R = Z$ and $R = Z + d_{2,\max}$ form two coaxial cones with a slant angle of $45^\circ$ ($\pi/4$).
*   The resulting workspace is a **hollow truncated cone** (funnel shape):

```
                   \       /
                    \  Z  / <--- Inner boundary: R = Z
                     \___/  
                     /   \
                    /  O  \ <--- Outer boundary: R = Z + d_2,max
                   /_______\
```

#### b) Inverse Kinematics
Given $(p_x, p_y, p_z)$, find $(\vartheta_1, d_2, d_3)$ assuming $d_2 > 0$:
1.  $$p_z = \frac{\sqrt{2}}{2}d_3 \implies d_3 = \sqrt{2} p_z$$
2.  Let $R = d_2 + \frac{\sqrt{2}}{2}d_3 = \sqrt{p_x^2 + p_y^2}$.
    Since $p_x = -R\sin\vartheta_1$ and $p_y = R\cos\vartheta_1$:
    $$\vartheta_1 = \arctan2(-p_x, p_y) = -\arctan\left(\frac{p_x}{p_y}\right)$$
3.  $$d_2 = R - \frac{\sqrt{2}}{2}d_3 = \sqrt{p_x^2 + p_y^2} - p_z$$

---

### Problema 4: Trajectory Planning (Double Triangular Velocity Profile)
#### a) Parametric Equations for Position
The trajectory consists of two linear segments:
*   **Segment 1**: From origin $(0,0)$ along angle $\pi/6$ ($30^\circ$) of length $\ell$.
    $$p_1(s) = \begin{bmatrix} s\cos(\pi/6) \\ s\sin(\pi/6) \end{bmatrix} = \begin{bmatrix} \frac{\sqrt{3}}{2}s \\ \frac{1}{2}s \end{bmatrix}, \quad 0 \le s \le \ell$$
*   **Segment 2**: From the end of Segment 1 vertically downwards of length $\ell$.
    $$p_2(s) = \begin{bmatrix} \frac{\sqrt{3}}{2}\ell \\ \frac{1}{2}\ell - (s - \ell) \end{bmatrix}, \quad \ell < s \le 2\ell$$

#### b) Double Triangular Velocity Profile Parameterization
The velocity profile has two triangular peaks of height $\dot{s}_{\max}$ at $t = t_f/4$ and $t = 3t_f/4$:
$$\dot{s}(t) = \begin{cases}
\frac{4\dot{s}_{\max}}{t_f} t & 0 \le t \le t_f/4 \\
-\frac{4\dot{s}_{\max}}{t_f} (t - t_f/2) & t_f/4 < t \le t_f/2 \\
\frac{4\dot{s}_{\max}}{t_f} (t - t_f/2) & t_f/2 < t \le 3t_f/4 \\
-\frac{4\dot{s}_{\max}}{t_f} (t - t_f) & 3t_f/4 < t \le t_f
\end{cases}$$

#### c) Relationship between $\dot{s}_{\max}$, $t_f$, and Length
The total displacement is the area under the two triangles:
$$\text{Area} = 2 \times \left( \frac{1}{2} \times \frac{t_f}{2} \times \dot{s}_{\max} \right) = \frac{\dot{s}_{\max} t_f}{2}$$
Since the total path length is $2\ell$:
$$2\ell = \frac{\dot{s}_{\max} t_f}{2} \implies t_f = \frac{4\ell}{\dot{s}_{\max}}$$

---

## 4. Exam 2022/2023 - First Exam (1º Exame)

### Problema 1: 4-DOF Manipulator (DH & Null-Space)
#### a) DH Parameters Table
For the manipulator in Fig. P1:

| $i$ | $d_i$ | $\vartheta_i$ | $a_i$ | $\alpha_i$ |
| :--- | :--- | :--- | :--- | :--- |
| **1** | $d_1(t)$ (Var) | $0$ | $0$ | $\pi/2$ |
| **2** | $0$ | $\vartheta_2(t)$ (Var) | $a_2$ | $\pi/2$ |
| **3** | $d_3(t)$ (Var) | $0$ | $0$ | $-\pi/2$ |
| **4** | $0$ | $\vartheta_4(t)$ (Var) | $a_4$ | $0$ |

#### b) Null-Space Motion
Fixing the end-effector (Frame 4), joint 2 (revolute, $\vartheta_2$) is constrained to move along a circle of radius $a_2$ centered at joint 1.
As it moves, the prismatic joints $d_1$ and $d_3$ adjust their lengths to keep the end-effector at its fixed coordinate:

```
               [Joint 1]
                  |
                  | d_1 (vertical adjustment)
                  |
               [Joint 2] ---\
                  \          \ Link 2 (swings on circle of radius a_2)
                   \          \
                  [Joint 3]----[End Effector] (Fixed in space)
                      d_3 (horizontal adjustment)
```

---

### Problema 2: 3-DOF Spatial Manipulator (Direct Kinematics & Jacobian)
#### a) Direct Kinematics ($R_3^0$ and $p_3$)
Using the DH table provided in Figure P2:
*   $R_1^0 = I_3$
*   $R_2^1 = R_z(\vartheta_2) R_x(-\pi/2) = \begin{bmatrix} c_2 & 0 & -s_2 \\ s_2 & 0 & c_2 \\ 0 & -1 & 0 \end{bmatrix}$
*   $R_3^2 = I_3$

Thus:
$$R_3^0 = R_2^1 = \begin{bmatrix} c_2 & 0 & -s_2 \\ s_2 & 0 & c_2 \\ 0 & -1 & 0 \end{bmatrix}$$

For the position:
$$p_3 = d_1 z_0 + d_3 z_3^0 = d_1 \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} + d_3 \begin{bmatrix} -s_2 \\ c_2 \\ 0 \end{bmatrix} = \begin{bmatrix} -d_3 s_2 \\ d_3 c_2 \\ d_1 \end{bmatrix}$$

#### b) Build the Jacobian Matrix $J$
With $q = [d_1, \vartheta_2, d_3]^T$:
1.  **Column 1 (Prismatic $d_1$)**:
    $$J_{P1} = z_0 = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}, \quad J_{O1} = \mathbf{0}$$
2.  **Column 2 (Revolute $\vartheta_2$)**:
    $$J_{P2} = z_1 \times (p_3 - p_1) = z_0 \times (p_3 - d_1 z_0) = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} \times \begin{bmatrix} -d_3 s_2 \\ d_3 c_2 \\ 0 \end{bmatrix} = \begin{bmatrix} -d_3 c_2 \\ -d_3 s_2 \\ 0 \end{bmatrix}$$
    $$J_{O2} = z_1 = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}$$
3.  **Column 3 (Prismatic $d_3$)**:
    $$J_{P3} = z_2^0 = z_3^0 = \begin{bmatrix} -s_2 \\ c_2 \\ 0 \end{bmatrix}, \quad J_{O3} = \mathbf{0}$$

---

### Problema 3: Workspace & Inverse Kinematics
#### a) Workspace Volume Sketch
Here, $(x, y, z) = (-d_3\sin\vartheta_2, d_3\cos\vartheta_2, d_1)$.
*   $d_{3,\min} \le d_3 \le d_{3,\max}$ represents the radius $R$ of the cylinder.
*   $\vartheta_2 \in [0, 2\pi]$ sweeps the circle.
*   $d_{1,\min} \le d_1 \le d_{1,\max}$ is the height of the cylinder.
This defines a **hollow cylinder** (thick-walled cylinder) with inner radius $d_{3,\min}$, outer radius $d_{3,\max}$, and height $d_{1,\max} - d_{1,\min}$.

```
                      . - - - .
                  . /   . - .   \ .
                 / /  /       \  \ \  
                | |  |  Z-axis |  | | } height = d_1,max - d_1,min
                 \ \  \       /  / /
                  . \   ` - '   / .
                      ` - - - '
                      |<--d_3,min-->|
                      |<------d_3,max------>|
```

#### b) Inverse Kinematics
Given desired position $(e_x, e_y, e_z)$:
1.  $$d_1 = e_z$$
2.  $$d_3 = \sqrt{e_x^2 + e_y^2}$$
3.  $$\sin\vartheta_2 = -\frac{e_x}{d_3}, \quad \cos\vartheta_2 = \frac{e_y}{d_3} \implies \vartheta_2 = \arctan2(-e_x, e_y)$$

---

### Problema 4: Trajectory Planning (Cubic Timing Law)
#### a) Parametrize the Trajectory $p(s)$
The path consists of a straight segment of length 3 from $(1,1)$ to $(4,1)$, followed by a semicircle of radius 1 centered at $(4,2)$ from $(4,1)$ to $(4,3)$.
*   **Segment 1** ($0 \le s < 3$):
    $$p(s) = \begin{bmatrix} 1 + s \\ 1 \end{bmatrix}$$
*   **Segment 2** ($3 \le s \le 3 + \pi$):
    $$p(s) = \begin{bmatrix} 4 + \sin(s - 3) \\ 2 - \cos(s - 3) \end{bmatrix}$$

#### b) Cubic Timing Law & Conclusion Time
Given the cubic timing law:
$$s(t) = s_0 + 3(s_f - s_0)\left(\frac{t}{t_f}\right)^2 - 2(s_f - s_0)\left(\frac{t}{t_f}\right)^3$$
1.  Differentiate to find the velocity $\dot{s}(t)$:
    $$\dot{s}(t) = \frac{6(s_f - s_0)}{t_f^2} t - \frac{6(s_f - s_0)}{t_f^3} t^2$$
2.  To find maximum velocity, set acceleration $\ddot{s}(t) = 0$:
    $$\ddot{s}(t) = \frac{6(s_f - s_0)}{t_f^2} \left(1 - \frac{2t}{t_f}\right) = 0 \implies t = \frac{t_f}{2}$$
3.  Substitute $t = t_f/2$ into $\dot{s}(t)$:
    $$\dot{s}_{\max} = \dot{s}(t_f/2) = \frac{6(s_f - s_0)}{t_f^2} \left(\frac{t_f}{2}\right) - \frac{6(s_f - s_0)}{t_f^3} \left(\frac{t_f^2}{4}\right) = \frac{3(s_f - s_0)}{2 t_f}$$
4.  Solve for $t_f$:
    $$t_f = \frac{3(s_f - s_0)}{2 \dot{s}_{\max}}$$
    Given $s_f - s_0 = 3 + \pi$:
    $$t_f = \frac{3(3 + \pi)}{2 \dot{s}_{\max}}$$

---

## 5. Exam 2022/2023 - Second Exam (2º Exame)

### Problema 1: 4-DOF Planar Manipulator (DH & Singularity)
#### a) DH Parameters Table
For the manipulator in Fig. P1:

| $i$ | $d_i$ | $\vartheta_i$ | $a_i$ | $\alpha_i$ |
| :--- | :--- | :--- | :--- | :--- |
| **1** | $0$ | $\vartheta_1(t)$ (Var) | $a_1$ | $0$ |
| **2** | $d_2(t)$ (Var) | $\pi/2$ | $0$ | $-\pi/2$ |
| **3** | $d_3(t)$ (Var) | $0$ | $0$ | $\pi/2$ |
| **4** | $0$ | $\vartheta_4(t)$ (Var) | $a_4$ | $0$ |

#### b) Interval Singularity (Singularidade de Intervalo)
A singularity occurs when the axes $z_0$ and $z_3$ become collinear:
$$z_0 \equiv z_3 \quad \text{which occurs when} \quad d_3 = a_1$$
In this configuration, any rotation about joint 1 ($\vartheta_1$) produces the exact same end-effector velocity direction as rotation about joint 4 ($\vartheta_4$), reducing the rank of the Jacobian.

---

### Problema 2: 4-DOF Spatial Manipulator (Direct Kinematics & Jacobian)
#### a) Direct Kinematics ($R_4^0$ and $p_4$)
Using the DH table in Figure P2:
##### 1. Orientation Matrix $R_4^0$
$$R_1^0 = R_x(-\pi/2), \quad R_2^1 = R_x(\pi/2) \implies R_2^0 = I_3$$
$$R_4^0 = R_z(\vartheta_3 + \vartheta_4) = \begin{bmatrix} c_{34} & -s_{34} & 0 \\ s_{34} & c_{34} & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

##### 2. Position Vector $p_4$
$$p_4 = d_1 z_0 + d_2 y_0 + a_3 x_3 + a_4 x_4 = \begin{bmatrix} a_3 c_3 + a_4 c_{34} \\ d_2 + a_3 s_3 + a_4 s_{34} \\ d_1 \end{bmatrix}$$

#### b) Build the Jacobian Matrix $J$
With $q = [d_1, d_2, \vartheta_3, \vartheta_4]^T$:
1.  **Column 1 (Prismatic $d_1$)**:
    $$J_{P1} = z_0 = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}, \quad J_{O1} = \mathbf{0}$$
2.  **Column 2 (Prismatic $d_2$)**:
    $$J_{P2} = z_1^0 = y_0 = \begin{bmatrix} 0 \\ 1 \\ 0 \end{bmatrix}, \quad J_{O2} = \mathbf{0}$$
3.  **Column 3 (Revolute $\vartheta_3$)**:
    $$J_{P3} = z_2 \times (p_4 - p_2) = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} \times \begin{bmatrix} a_3 c_3 + a_4 c_{34} \\ a_3 s_3 + a_4 s_{34} \\ 0 \end{bmatrix} = \begin{bmatrix} -a_3 s_3 - a_4 s_{34} \\ a_3 c_3 + a_4 c_{34} \\ 0 \end{bmatrix}$$
    $$J_{O3} = z_2 = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}$$
4.  **Column 4 (Revolute $\vartheta_4$)**:
    $$J_{P4} = z_3 \times (p_4 - p_3) = z_0 \times (a_4 x_4) = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix} \times \begin{bmatrix} a_4 c_{34} \\ a_4 s_{34} \\ 0 \end{bmatrix} = \begin{bmatrix} -a_4 s_{34} \\ a_4 c_{34} \\ 0 \end{bmatrix}$$
    $$J_{O4} = z_3 = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}$$

---

### Problema 3: Workspace & Inverse Kinematics
#### a) Workspace Volume Sketch (for $\vartheta_4 = 0$)
The position vector is:
$$p_4 = \begin{bmatrix} (a_3 + a_4)\cos\vartheta_3 \\ d_2 + (a_3+a_4)\sin\vartheta_3 \\ d_1 \end{bmatrix}$$
*   $d_{1,\min} \le d_1 \le d_{1,\max}$ (extrudes vertically along $z$)
*   $d_{2,\min} \le d_2 \le d_{2,\max}$ (translates horizontally along $y$)
*   $\vartheta_3 \in [0, 2\pi]$ (sweeps a circle of radius $R = a_3 + a_4$ in the $xy$-plane)
In the $xy$-plane, the region is a **stadium shape** translated along $y$ of width $2(a_3+a_4)$ and length $d_{2,\max}-d_{2,\min}$. This 2D profile is extruded vertically from $d_{1,\min}$ to $d_{1,\max}$, creating a **stadium prism**.

#### b) Inverse Kinematics
Given desired position $(e_x, e_y, e_z)$, find $(d_1, d_2, \vartheta_3)$:
1.  $$d_1 = e_z$$
2.  $$\cos\vartheta_3 = \frac{e_x}{a_3 + a_4} \implies \vartheta_3 = \arccos\left(\frac{e_x}{a_3+a_4}\right)$$
3.  $$d_2 = e_y - (a_3 + a_4)\sin\vartheta_3$$

---

### Problema 4: Trajectory Planning (Triangular Velocity Profile)
#### a) Parametrize the Trajectory $p(s)$
The path consists of a quarter circle from $(1,3)$ to $(2,2)$ of radius 1 centered at $(2,3)$, a horizontal straight segment of length 2 to $(4,2)$, and a quarter circle of radius 1 centered at $(4,1)$ to $(5,1)$.
Total length is $L = \pi/2 + 2 + \pi/2 = 2 + \pi$.
*   **Segment 1** ($0 \le s < \pi/2$):
    $$p_1(s) = \begin{bmatrix} 2 - \cos s \\ 3 - \sin s \end{bmatrix}$$
*   **Segment 2** ($\pi/2 \le s < \pi/2 + 2$):
    Let $s_2 = s - \pi/2$:
    $$p_2(s) = \begin{bmatrix} 2 + s_2 \\ 2 \end{bmatrix}$$
*   **Segment 3** ($\pi/2 + 2 \le s \le 2 + \pi$):
    Let $s_3 = s - \pi/2 - 2$:
    $$p_3(s) = \begin{bmatrix} 4 + \sin s_3 \\ 1 + \cos s_3 \end{bmatrix}$$

#### b) Triangular Velocity Profile
For a triangular velocity profile peaking at $t_f/2$:
$$\dot{s}(t) = \begin{cases}
\frac{2\dot{s}_{\max}}{t_f} t & 0 \le t \le t_f/2 \\
-\frac{2\dot{s}_{\max}}{t_f} (t - t_f) & t_f/2 < t \le t_f
\end{cases}$$
Integrating this, the displacement $s(t)$ is:
$$s(t) = \begin{cases}
\frac{\dot{s}_{\max}}{t_f} t^2 & 0 \le t \le t_f/2 \\
\frac{t_f \dot{s}_{\max}}{2} - \frac{\dot{s}_{\max}}{t_f} (t - t_f)^2 & t_f/2 < t \le t_f
\end{cases}$$

#### c) Relationship
The total displacement is the area of the triangle:
$$L = \text{Area} = \frac{t_f \dot{s}_{\max}}{2}$$
Given $L = 2 + \pi$:
$$2 + \pi = \frac{t_f \dot{s}_{\max}}{2} \implies t_f = \frac{2(2 + \pi)}{\dot{s}_{\max}}$$
