# Step 4: Geometric Jacobian & Singularities

This guide explains the code for Step 4, where we compute the **Geometric Jacobian** of the KUKA LBR MED robot and analyze what happens during **singularities**.

---

## 1. What is a Jacobian?

In Direct Kinematics, we calculated the **position** $p(q)$ and **orientation** $R(q)$ of the robot's end-effector based on its joint angles $q$.

The **Jacobian** $J(q)$ is simply the derivative of that relationship. It tells us how the end-effector's **velocity** changes based on the joint **velocities** $\dot{q}$:

$$ v_{ee} = J(q) \cdot \dot{q} $$

Where:
- $v_{ee}$ is a 6D vector (3 linear velocities $v_x, v_y, v_z$, and 3 angular velocities $\omega_x, \omega_y, \omega_z$).
- $\dot{q}$ is a 7D vector of joint speeds.
- $J(q)$ is a $6 \times 7$ matrix.

### How is it calculated in code?
In `generate_jacobian_library.m`, we build the matrix column-by-column. For a rotational joint $i$, its column in the Jacobian is:

$$ J_i = \begin{bmatrix} z_{i-1} \times (p_e - p_{i-1}) \\ z_{i-1} \end{bmatrix} $$

- $z_{i-1}$ is the axis the joint rotates around (e.g., the local Z-axis of the previous frame). This directly controls the angular velocity $\omega$.
- $(p_e - p_{i-1})$ is the vector from the joint to the end-effector. The cross product with the rotation axis gives the linear velocity $v$ (just like $v = \omega \times r$ in physics).

The script runs this math symbolically and saves it as a Simulink block in `Kuka_Lib.slx`.

---

## 2. Validation by Numerical Differentiation

How do we know the massive mathematical formula for the Jacobian is correct?

We can test it using a concept from basic calculus: **Numerical Differentiation**.
If the Jacobian is the derivative of position, then multiplying it by joint speed should give us the exact same result as tracking the position over time and calculating speed manually ($Speed = \frac{\Delta Distance}{\Delta Time}$).

In `validate_jacobian.m` (Part A):
1. We simulate the robot moving in a smooth wave (sine wave) over time $t$.
2. **Method 1 (Analytical):** We plug the angles $q$ into our Jacobian matrix $J(q)$, and multiply by the joint speeds $\dot{q}$.
3. **Method 2 (Numerical):** We plug the angles $q$ into our Direct Kinematics to find the position $p$ at time $t_1$ and $t_2$, and calculate $v = \frac{p(t_2) - p(t_1)}{dt}$.
4. The script compares the two velocities. They match almost perfectly (difference $< 0.0001$ m/s), proving our Jacobian formula is flawless!

---

## 3. Kinematic Singularities (Rank Analysis)

A robot has 6 spatial degrees of freedom (3 translation, 3 rotation). The KUKA arm has 7 joints, meaning it has "redundancy" (it can reach the same point in multiple ways).

However, in certain poses, the robot **loses** the ability to move in a specific direction. These are called **Singularities**.

Mathematically, a matrix that allows movement in all 6 directions has a **Rank of 6**. At a singularity, the Jacobian matrix loses a rank (e.g., drops to 5). 

In `validate_jacobian.m` (Part B), we test 4 poses:

### 1. Generic Pose
A normal, bent arm configuration.
- **Result:** Rank = 6. The robot can freely move in any direction (6 Task-Space DOFs, plus 1 internal Redundancy DOF).

### 2. Elbow Singularity (Arm Fully Stretched)
If $q_4 = 0$, the arm is completely straight.
- **Physical effect:** The robot cannot push its hand further outward because there are no joints left to extend. It loses 1 **Task-Space** Degree of Freedom (movement along the arm axis).
- **Result:** Rank drops to **5**.

### 3. Shoulder Singularity (Arm Straight Up)
If $q_2 = 0$, the elbow is directly above the shoulder.
- **Physical effect:** Joint 1 (base rotation) and Joint 3 (arm roll) become perfectly aligned along the vertical axis. Because two joints are doing the exact same job, 1 **Redundancy** Degree of Freedom is wasted.
- **Result:** Rank is still **6**! The hand can still move in all 6 spatial directions, the robot just loses its ability to freely reconfigure its internal posture.

### 4. Wrist Singularity
If $q_6 = 0$, the wrist is completely straight.
- **Physical effect:** Joint 5 and Joint 7 axes become perfectly aligned. Just like the shoulder, two joints are spinning the hand in the exact same axis, wasting 1 **Redundancy** Degree of Freedom.
- **Result:** Rank is still **6**! No spatial mobility is lost, just internal redundancy.
