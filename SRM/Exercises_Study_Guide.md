# Robotic Systems in Manipulation (Sistemas Robóticos em Manipulação)
## Exercises Study Guide: Problems Set 1 to 11

This study guide covers the entire problem syllabus of the **Robotic Systems in Manipulation** course, structured into 11 distinct problem sets. Each section includes a quick theoretical reference of the concepts, followed by step-by-step mathematical derivations of core representative problems, complete with ASCII visual aids.

---

## Set 1: Rotations and Translations

### 1.1 Core Theory Reference
To change the representation of a vector $r$ from frame $\beta$ to frame $\alpha$:
$$r^\alpha = R_\beta^\alpha r^\beta$$
Where the rotation matrix $R_\beta^\alpha$ is formed by the coordinate representation of the axes of frame $\beta$ inside frame $\alpha$:
$$R_\beta^\alpha = \begin{bmatrix} x_\beta^\alpha & y_\beta^\alpha & z_\beta^\alpha \end{bmatrix}$$
Since rotation matrices are orthonormal:
$$(R_\beta^\alpha)^{-1} = (R_\beta^\alpha)^T = R_\alpha^\beta$$
Elementary rotations about the coordinate axes are:
$$R_x(\phi) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & \cos\phi & -\sin\phi \\ 0 & \sin\phi & \cos\phi \end{bmatrix}, \quad R_y(\theta) = \begin{bmatrix} \cos\theta & 0 & \sin\theta \\ 0 & 1 & 0 \\ -\sin\theta & 0 & \cos\theta \end{bmatrix}, \quad R_z(\psi) = \begin{bmatrix} \cos\psi & -\sin\psi & 0 \\ \sin\psi & \cos\psi & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

For translation, if frame $\beta$ is translated by $r_{\alpha,\beta}$ relative to frame $\alpha$:
$$r^\alpha = r_{\alpha,\beta} + R_\beta^\alpha r^\beta$$

### 1.2 Step-by-Step Problem: Vector Projections & Rotations
Let frame 2 be rotated relative to frame 1 around the $z$-axis by an angle $\vartheta$:

```
        Y_1    Y_2
         ^    /
         |   / 
         |  / 
         | /  
         |/_____\ Vtheta
         O-------> X_1
          \      
           \ 
            v X_2
```

#### Step 1: Express the axes of frame 2 inside frame 1
Projecting unit vectors $x_2$ and $y_2$ onto the axes of frame 1 ($x_1$, $y_1$):
*   $x_2^1 = \begin{bmatrix} x_2 \cdot x_1 \\ x_2 \cdot y_1 \\ x_2 \cdot z_1 \end{bmatrix} = \begin{bmatrix} \cos\vartheta \\ \sin\vartheta \\ 0 \end{bmatrix}$
*   $y_2^1 = \begin{bmatrix} y_2 \cdot x_1 \\ y_2 \cdot y_1 \\ y_2 \cdot z_1 \end{bmatrix} = \begin{bmatrix} \cos(\vartheta + \pi/2) \\ \sin(\vartheta + \pi/2) \\ 0 \end{bmatrix} = \begin{bmatrix} -\sin\vartheta \\ \cos\vartheta \\ 0 \end{bmatrix}$

#### Step 2: Build the Rotation Matrix
$$R_2^1 = \begin{bmatrix} x_2^1 & y_2^1 & z_2^1 \end{bmatrix} = \begin{bmatrix} \cos\vartheta & -\sin\vartheta & 0 \\ \sin\vartheta & \cos\vartheta & 0 \\ 0 & 0 & 1 \end{bmatrix}$$

---

## Set 2: Direct Kinematics (Euler Angles)

### 2.1 Core Theory Reference
An Euler angle sequence represents any 3D rotation using three sequential rotations about moving axes. For a **XYX Euler Sequence** ($\phi, \theta, \psi$):
$$R(\phi, \theta, \psi) = R_x(\phi) R_y(\theta) R_x(\psi)$$

### 2.2 Step-by-Step Problem: Inverse Euler Angles
Given the target orientation matrix:
$$R_e^b = \begin{bmatrix}
r_{11} & r_{12} & r_{13} \\
r_{21} & r_{22} & r_{23} \\
r_{31} & r_{32} & r_{33}
\end{bmatrix} = \begin{bmatrix}
0.9659 & 0.2241 & 0.1294 \\
0.1830 & -0.2380 & -0.9539 \\
-0.1830 & 0.9451 & -0.2709
\end{bmatrix}$$
Solve for $\phi$, $\theta$, $\psi$ of a XYX sequence.

#### Step 1: Multiply out the analytical rotation matrix
$$R_{XYX} = R_x(\phi) R_y(\theta) R_x(\psi) = \begin{bmatrix} 1 & 0 & 0 \\ 0 & c_\phi & -s_\phi \\ 0 & s_\phi & c_\phi \end{bmatrix} \begin{bmatrix} c_\theta & 0 & s_\theta \\ 0 & 1 & 0 \\ -s_\theta & 0 & c_\theta \end{bmatrix} \begin{bmatrix} 1 & 0 & 0 \\ 0 & c_\psi & -s_\psi \\ 0 & s_\psi & c_\psi \end{bmatrix}$$
Multiplying gives:
$$R_{XYX} = \begin{bmatrix}
c_\theta & s_\theta s_\psi & s_\theta c_\psi \\
s_\phi s_\theta & c_\phi c_\psi - s_\phi c_\theta s_\psi & -c_\phi s_\psi - s_\phi c_\theta c_\psi \\
-c_\phi s_\theta & s_\phi c_\psi + c_\phi c_\theta s_\psi & -s_\phi s_\psi + c_\phi c_\theta c_\psi
\end{bmatrix}$$

#### Step 2: Extract the angles from numeric comparisons
1.  From the $(1,1)$ entry:
    $$\cos\theta = r_{11} = 0.9659 \implies \theta = \arccos(0.9659) \approx \pm 15.0^\circ \approx \pm \frac{\pi}{12} \text{ rad}$$
2.  If we choose $\theta > 0$ ($\theta \approx \pi/12$):
    *   $\sin\theta = 0.2588$
    *   From $(2,1)$ entry: $s_\phi s_\theta = r_{21} \implies \sin\phi = \frac{0.1830}{0.2588} = 0.7071$
    *   From $(3,1)$ entry: $-c_\phi s_\theta = r_{31} \implies \cos\phi = \frac{0.1830}{0.2588} = 0.7071$
    *   Thus, $\phi = \arctan2(\sin\phi, \cos\phi) = \pi/4 \text{ rad} = 45^\circ$.
3.  From the $(1,2)$ and $(1,3)$ entries:
    *   $s_\theta s_\psi = r_{12} \implies \sin\psi = \frac{0.2241}{0.2588} = 0.8660$
    *   $s_\theta c_\psi = r_{13} \implies \cos\psi = \frac{0.1294}{0.2588} = 0.5000$
    *   Thus, $\psi = \arctan2(\sin\psi, \cos\psi) = \pi/3 \text{ rad} = 60^\circ$.

We verified the solution: $(\phi, \theta, \psi) \approx (\pi/4, \pi/12, \pi/3)$.

---

## Set 3: Denavit-Hartenberg Parametrization

### 3.1 Core Theory Reference
The DH convention places frame axes such that we can easily represent coordinate transfers sequentially. The key rules are:
*   $z_{i-1}$ is along the axis of joint $i$.
*   $x_i$ is normal to both $z_{i-1}$ and $z_i$.

### 3.2 Step-by-Step Problem: Spherical Wrist Frame Assignment
A spherical wrist has 3 orthogonal revolute joints intersecting at a single point (the wrist center):

```
                       Z_4, Z_5, Z_6
                             |
                             O (Intersection point of all 3 axes)
                            / \
                           /   \
                         X_4   Y_4
```

To assign frames and build the table:
*   Joint 4 axis is $z_3$.
*   Joint 5 axis is $z_4$, perpendicular to $z_3$.
*   Joint 6 axis is $z_5$, perpendicular to $z_4$.
Since all axes intersect at a single point $O$, the link lengths $a_4 = a_5 = a_6 = 0$.
The DH Table is:

| Link | $d_i$ | $\vartheta_i$ | $a_i$ | $\alpha_i$ |
| :--- | :--- | :--- | :--- | :--- |
| **4** | $0$ | $\vartheta_4$ | $0$ | $-\pi/2$ |
| **5** | $0$ | $\vartheta_5$ | $0$ | $\pi/2$ |
| **6** | $d_6$ | $\vartheta_6$ | $0$ | $0$ |

---

## Set 4: Inverse Kinematics (Kinematic Decoupling)

### 4.1 Core Theory Reference
For 6-DOF manipulators with a spherical wrist, we decouple the inverse kinematics problem into two simpler subsets:
1.  **Inverse Position**: Find the wrist center coordinates $W = p_e - d_6 z_e$, and use the first 3 joint coordinates to move the arm to $W$.
2.  **Inverse Orientation**: Use the remaining 3 joint coordinates (the wrist joints) to match the final orientation $R_6^3 = (R_3^0)^T R_e$.

### 4.2 Step-by-Step Problem: Decoupling a Circular Manipulator
Consider a manipulator where the wrist position is given by:
$$W = \begin{bmatrix} e_x - a_3 \cos\phi \\ e_y - a_3 \sin\phi \\ 0 \end{bmatrix}$$
And the structural position equations of the arm are:
$$W_x = a_1 \cos\theta_1 + d_2 \cos(\theta_1 - \pi/2) = a_1 \cos\theta_1 + d_2 \sin\theta_1$$
$$W_y = a_1 \sin\theta_1 - d_2 \cos\theta_1$$

```
                   [Joint 2, d_2]------> [Wrist Center, W]
                        /
                       / Link 1 (a_1)
                      /
                     O [Joint 1, theta_1]
```

#### Step 1: Solve for $d_2$
Square both equations:
$$W_x^2 + W_y^2 = (a_1 \cos\theta_1 + d_2 \sin\theta_1)^2 + (a_1 \sin\theta_1 - d_2 \cos\theta_1)^2$$
Expanding and simplifying:
$$W_x^2 + W_y^2 = a_1^2(\cos^2\theta_1 + \sin^2\theta_1) + d_2^2(\sin^2\theta_1 + \cos^2\theta_1) = a_1^2 + d_2^2$$
$$d_2 = \pm \sqrt{W_x^2 + W_y^2 - a_1^2}$$

#### Step 2: Solve for $\theta_1$
Let $\alpha = \arctan2(W_y, W_x)$ and $\beta = \arctan2(d_2, a_1)$.
From the geometry:
$$\theta_1 = \alpha + \beta \quad \text{or} \quad \theta_1 = \alpha - \beta$$

---

## Set 5: Jacobian Matrix

### 5.1 Core Theory Reference
The **Geometric Jacobian** maps joint velocities directly to linear and angular velocities:
$$\begin{bmatrix} v_e \\ \omega_e \end{bmatrix} = J(q) \dot{q}$$
The columns are:
*   $J_i = \begin{bmatrix} z_{i-1} \\ \mathbf{0} \end{bmatrix}$ (Prismatic)
*   $J_i = \begin{bmatrix} z_{i-1} \times (p_e - p_{i-1}) \\ z_{i-1} \end{bmatrix}$ (Revolute)

### 5.2 Step-by-Step Problem: Geometric Jacobian of 3-Link Manipulator
Given the position of a 3-link manipulator:
$$p_3 = \begin{bmatrix} (a_2 + a_3 \cos\theta_3) \cos\theta_2 \\ a_3 \sin\theta_3 \\ d_1 - (a_2 + a_3 \cos\theta_3) \sin\theta_2 \end{bmatrix}$$
With joint variables $q = [d_1, \theta_2, \theta_3]^T$.

#### Step 1: Derive Columns
*   **Joint 1 (Prismatic, along $z$-axis)**:
    $$J_{P1} = z_0 = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}, \quad J_{O1} = \mathbf{0}$$
*   **Joint 2 (Revolute, about $y$-axis)**:
    $$J_{P2} = z_1 \times (p_3 - p_1) = \begin{bmatrix} 0 \\ 1 \\ 0 \end{bmatrix} \times \begin{bmatrix} (a_2 + a_3 c_3) c_2 \\ a_3 s_3 \\ -(a_2 + a_3 c_3) s_2 \end{bmatrix} = \begin{bmatrix} -(a_2 + a_3 c_3) s_2 \\ 0 \\ -(a_2 + a_3 c_3) c_2 \end{bmatrix}$$
    $$J_{O2} = \begin{bmatrix} 0 \\ 1 \\ 0 \end{bmatrix}$$
*   **Joint 3 (Revolute, about $y$-axis)**:
    $$J_{P3} = z_2 \times (p_3 - p_2) = \begin{bmatrix} 0 \\ 1 \\ 0 \end{bmatrix} \times \begin{bmatrix} a_3 c_3 c_2 \\ a_3 s_3 \\ -a_3 c_3 s_2 \end{bmatrix} = \begin{bmatrix} -a_3 s_3 c_2 \\ a_3 c_3 \\ -a_3 s_3 s_2 \end{bmatrix}$$
    *(Note: The negative sign in the $z$-component of $J_{P3}$ arises from coordinates mapping).*
    $$J_{O3} = \begin{bmatrix} 0 \\ 1 \\ 0 \end{bmatrix}$$

---

## Set 6: Singularities & Redundancy

### 6.1 Core Theory Reference
*   **Singularities**: Configuration states where the rank of $J(q)$ drops:
    $$\det(J_{3\times 3}(q)) = 0 \quad \text{or} \quad \det(J_P J_P^T) = 0$$
*   **Redundancy**: When $n > m$, we can use the **Pseudoinverse** $J^\dagger = J^T(J J^T)^{-1}$ to find joint velocities that minimize kinetic energy:
    $$\dot{q} = J^\dagger v_e + (I - J^\dagger J) \dot{q}_0$$
    Where $(I - J^\dagger J) \dot{q}_0$ projects any arbitrary joint velocity $\dot{q}_0$ into the **null-space** (meaning it creates internal link motion without moving the end-effector).

### 6.2 Step-by-Step Problem: Singularity Analysis of a 3-DOF Arm
Let the Jacobian of a 3-link planar elbow arm be:
$$J(q) = \begin{bmatrix} -a_1 s_1 - a_2 s_{12} & -a_2 s_{12} & 0 \\ a_1 c_1 + a_2 c_{12} & a_2 c_{12} & 0 \\ 1 & 1 & 1 \end{bmatrix}$$

#### Step 1: Find the determinant
$$\det(J) = 1 \cdot [(-a_1 s_1 - a_2 s_{12})(a_2 c_{12}) - (-a_2 s_{12})(a_1 c_1 + a_2 c_{12})]$$
$$\det(J) = -a_1 a_2 s_1 c_{12} - a_2^2 s_{12} c_{12} + a_1 a_2 s_{12} c_1 + a_2^2 s_{12} c_{12}$$
$$\det(J) = a_1 a_2 (s_{12} c_1 - c_{12} s_1) = a_1 a_2 \sin(\theta_{12} - \theta_1) = a_1 a_2 \sin\theta_2$$

#### Step 2: Identify Singularity Conditions
$$\det(J) = 0 \implies \sin\theta_2 = 0 \implies \theta_2 = 0 \quad \text{or} \quad \theta_2 = \pi$$
*   **$\theta_2 = 0$**: Fully extended arm (boundary singularity).
*   **$\theta_2 = \pi$**: Fully folded arm (boundary singularity).

---

## Set 7: Static Forces & Torque Duality

### 7.1 Core Theory Reference
By the Principle of Virtual Work, the relationship between joint torques $\tau$ and the external generalized force/torque $F$ applied at the end-effector is:
$$\tau = J^T(q) F$$
Where $F = \begin{bmatrix} f_e \\ \mu_e \end{bmatrix}$.

### 7.2 Step-by-Step Problem: Torques under End-Effector Load
Consider a 2-DOF planar manipulator with:
$$J^T(q) = \begin{bmatrix} -a_1 s_1 - a_2 s_{12} & a_1 c_1 + a_2 c_{12} \\ -a_2 s_{12} & a_2 c_{12} \end{bmatrix}$$
Find the joint torques $\tau$ required to resist a purely horizontal force $F = \begin{bmatrix} f_x \\ 0 \end{bmatrix}$ acting at the tip.

#### Step 1: Matrix Multiplication
$$\tau = J^T F = \begin{bmatrix} -a_1 s_1 - a_2 s_{12} & a_1 c_1 + a_2 c_{12} \\ -a_2 s_{12} & a_2 c_{12} \end{bmatrix} \begin{bmatrix} f_x \\ 0 \end{bmatrix}$$
$$\tau_1 = -(a_1 \sin\theta_1 + a_2 \sin(\theta_1 + \theta_2)) f_x$$
$$\tau_2 = -a_2 \sin(\theta_1 + \theta_2) f_x$$

These torques directly balance the moment arms created by the horizontal force.

---

## Set 8: Trajectory Planning

### 8.1 Core Theory Reference
For point-to-point joint moves, **Cubic Polynomials** are used to interpolate joint positions while ensuring smooth velocities (zero velocity at boundaries):
$$\theta(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3$$

### 8.2 Step-by-Step Problem: Cubic Polynomial Coefficients
Find the coefficients for a joint trajectory starting at $\theta(0) = \theta_0$ with velocity $\dot{\theta}(0) = 0$ and ending at time $t_f$ at position $\theta(t_f) = \theta_f$ with velocity $\dot{\theta}(t_f) = 0$.

#### Step 1: Set up the equations
1.  $\theta(0) = a_0 \implies a_0 = \theta_0$
2.  $\dot{\theta}(t) = a_1 + 2 a_2 t + 3 a_3 t^2 \implies \dot{\theta}(0) = a_1 \implies a_1 = 0$
3.  $\theta(t_f) = \theta_0 + a_2 t_f^2 + a_3 t_f^3 = \theta_f$
4.  $\dot{\theta}(t_f) = 2 a_2 t_f + 3 a_3 t_f^2 = 0 \implies a_2 = -\frac{3}{2} a_3 t_f$

#### Step 2: Solve the linear system
Substitute $a_2$ into the third equation:
$$\theta_0 + \left(-\frac{3}{2} a_3 t_f\right) t_f^2 + a_3 t_f^3 = \theta_f \implies -\frac{1}{2} a_3 t_f^3 = \theta_f - \theta_0$$
$$a_3 = -\frac{2(\theta_f - \theta_0)}{t_f^3}$$
$$a_2 = \frac{3(\theta_f - \theta_0)}{t_f^2}$$

The final trajectory equation is:
$$\theta(t) = \theta_0 + \frac{3(\theta_f - \theta_0)}{t_f^2} t^2 - \frac{2(\theta_f - \theta_0)}{t_f^3} t^3$$

---

## Set 9: Manipulator Dynamics

### 9.1 Core Theory Reference
The Euler-Lagrange equations of motion are:
$$\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{q}_i}\right) - \frac{\partial \mathcal{L}}{\partial q_i} = \tau_i$$
Where the Lagrangian $\mathcal{L} = K - P$ (Kinetic Energy minus Potential Energy).

### 9.2 Step-by-Step Problem: 1-Link Revolute Joint Dynamics
Consider a single-link robotic arm of length $L$, mass $m$ concentrated at the tip, rotating in a vertical plane:

```
                  O [Joint 1, theta]
                  \
                   \ Link length L, Mass m
                    \
                     * (Mass m)
```

#### Step 1: Formulate Energy Terms
*   **Kinetic Energy $K$**:
    $$K = \frac{1}{2} m v^2 = \frac{1}{2} m (L\dot{\theta})^2 = \frac{1}{2} m L^2 \dot{\theta}^2$$
*   **Potential Energy $P$**: (taking the horizontal plane through the joint as reference, positive upwards)
    $$P = -mgL\cos\theta$$
*   **Lagrangian $\mathcal{L}$**:
    $$\mathcal{L} = K - P = \frac{1}{2} m L^2 \dot{\theta}^2 + mgL\cos\theta$$

#### Step 2: Apply Euler-Lagrange Equations
1.  Compute derivatives:
    $$\frac{\partial \mathcal{L}}{\partial \dot{\theta}} = m L^2 \dot{\theta} \implies \frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{\theta}}\right) = m L^2 \ddot{\theta}$$
    $$\frac{\partial \mathcal{L}}{\partial \theta} = -mgL\sin\theta$$
2.  Assemble equation of motion:
    $$m L^2 \ddot{\theta} + mgL\sin\theta = \tau$$

---

## Set 10: Joint Space Control

### 10.1 Core Theory Reference
For a dynamic system $M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau$, we design controllers in joint space:
*   **PD Control with Gravity Compensation**:
    $$\tau = K_p (q_d - q) + K_d (\dot{q}_d - \dot{q}) + g(q)$$
    This structure reduces the non-linear dynamics problem to a simple linear error system: $M \ddot{e} + K_d \dot{e} + K_p e = 0$.

### 10.2 Step-by-Step Problem: Controller Tuning
For a system with inertia $M = 2 \text{ kg}\cdot\text{m}^2$, design a PD controller such that the closed-loop system is critically damped with a natural frequency $\omega_n = 10 \text{ rad/s}$.

#### Step 1: Setup Characteristic Equation
The closed loop system error equation is:
$$\ddot{e} + \frac{K_d}{M} \dot{e} + \frac{K_p}{M} e = 0$$
For critical damping ($\zeta = 1$):
$$\ddot{e} + 2\omega_n \dot{e} + \omega_n^2 e = 0$$

#### Step 2: Calculate gains
1.  Match coefficients for $K_p$:
    $$\frac{K_p}{M} = \omega_n^2 \implies K_p = M \omega_n^2 = 2 \times (10)^2 = 200 \text{ N}\cdot\text{m/rad}$$
2.  Match coefficients for $K_d$:
    $$\frac{K_d}{M} = 2\omega_n \implies K_d = 2 M \omega_n = 2 \times 2 \times 10 = 40 \text{ N}\cdot\text{m}\cdot\text{s/rad}$$

---

## Set 11: Operational Space & Force Control

### 11.1 Core Theory Reference
When interacting with the environment, we use **Impedance Control** to establish a mass-spring-damper relationship between the contact force $f_e$ and the position error $x_d - x$:
$$M_d \ddot{e} + K_d \dot{e} + K_p e = f_e$$

```
    [Manipulator End Effector] ----> K_e (Environment Stiffness) ----| [Wall]
```

### 11.2 Step-by-Step Problem: Active Stiffness Design
Design a controller to maintain a constant contact force $f_d = 10 \text{ N}$ against a wall of stiffness $K_e = 1000 \text{ N/m}$.

#### Step 1: Model the interaction
At static equilibrium:
$$f_e = K_e (x - x_{\text{wall}})$$
The steady-state position error is:
$$x - x_d = -\frac{f_d}{K_p} \implies x = x_d - \frac{f_d}{K_p}$$

#### Step 2: Compute desired setpoint $x_d$
Equating the forces:
$$f_d = K_e \left(x_d - \frac{f_d}{K_p} - x_{\text{wall}}\right)$$
Assuming the stiffness gain $K_p$ is set to $2000 \text{ N/m}$:
$$10 = 1000 \left(x_d - \frac{10}{2000} - x_{\text{wall}}\right) \implies 0.01 = x_d - 0.005 - x_{\text{wall}}$$
$$x_d - x_{\text{wall}} = 0.015 \text{ m} = 1.5 \text{ cm}$$

To maintain a steady $10\text{ N}$ force, the controller must command a target position $1.5\text{ cm}$ inside the wall.
