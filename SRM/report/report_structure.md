# KUKA LBR MED — Report Structure (Results-Focused)

---

## 1. Introduction *(~0.5 pages)*
A single paragraph setting the context: what the robot is, what the project implemented, and what the report will demonstrate. No background theory.

---

## 2. D-H Table & Forward Kinematics *(~1 page)*

**Implementation:** The KUKA LBR MED's 7 joints were modelled using the Standard Denavit-Hartenberg convention. A key design decision was aligning the origins of Frame 0 and Frame 1 (setting $d_1 = 0$), which eliminates an unnecessary offset and simplifies the kinematic chain. The D-H table was encoded in `KukaLBR.m` and the global transformation $T_0^7 = A_1 \cdot A_2 \cdots A_7$ was computed symbolically in `generate_kuka_library.m`, producing the `kuka_direct_kinematics.m` function. Three physically meaningful configurations were selected for validation in `validate_direct_kinematics.m`.

**D-H Table:**
| Joint | $d$ (m) | $\theta$ | $a$ (m) | $\alpha$ (rad) |
|---|---|---|---|---|
| 1 | 0 | $q_1$ | 0 | $+\pi/2$ |
| 2 | 0 | $q_2$ | 0 | $-\pi/2$ |
| 3 | 0.400 | $q_3$ | 0 | $-\pi/2$ |
| 4 | 0 | $q_4$ | 0 | $+\pi/2$ |
| 5 | 0.400 | $q_5$ | 0 | $+\pi/2$ |
| 6 | 0 | $q_6$ | 0 | $-\pi/2$ |
| 7 | 0.126 | $q_7$ | 0 | 0 |

### Results Table — Key Configurations
| Configuration | Expected $p_z$ (m) | DK Result $p_z$ (m) | Error (mm) |
|---|---|---|---|
| Home (all zeros) | 0.926 | … | … |
| $q_2 = 90°$ Shoulder | … | … | … |
| L-shape | … | … | … |

**Discussion:** What do the zero errors confirm? Does the DK correctly encode the robot's geometry as described in the hand-drawn notes?

---

## 3. Analytical Inverse Kinematics *(~1 page)*

**Implementation:** A closed-form IK solver was implemented in `inverse_kinematics.m` exploiting kinematic decoupling — the arm (joints 1–4) is solved geometrically to place the wrist centre, and the wrist orientation (joints 5–7) is extracted analytically using Z-Y-Z Euler decomposition. The robot's 7-DOF redundancy is resolved by parameterising the elbow angle as a fixed input, reducing the problem to a unique geometric solution per elbow configuration. Reachability is enforced by clamping the cosine argument to $[-1, 1]$ before calling `acos`. Validation was performed in `validate_inverse_kinematics.m` using a round-trip test: the target pose is fed into IK, and the resulting joint angles are evaluated through Direct Kinematics to recover the original pose.

### Results Table — IK Round-Trip Test
| Test | Target $p$ (m) | Reachable? | IK $\rightarrow$ DK position error (mm) |
|---|---|---|---|
| Home | … | Yes | … |
| L-shape | … | Yes | … |
| Out of reach | … | No | — |

**Graph:** Bar chart showing position and orientation error for each test configuration.

**Discussion:** How accurate is the closed-form solution? Where does it fail (singularity configurations, unreachable workspace)? What does this tell us about the limitations of pure analytical IK for a clinical robot like the LBR MED?

---

## 4. Geometric Jacobian & Singularity Analysis *(~1.5 pages)*

**Implementation:** The $6 \times 7$ Geometric Jacobian was derived symbolically in `generate_jacobian_library.m`. For each joint $i$, the Jacobian column is built as $J_i = [z_{i-1} \times (p_e - p_{i-1}); \; z_{i-1}]$, where $z_{i-1}$ is the rotation axis and $p_e - p_{i-1}$ is the lever arm to the end-effector. The result was simplified symbolically and exported as `jacobian_kuka.m`. Validation was performed in `validate_jacobian.m` against a smooth multi-joint sine-wave trajectory, and four specific poses were tested to characterise singularities.

### 4.1 Numerical Validation
**Graph:** Line plot of Numerical velocity ($\Delta p / \Delta t$) vs. Analytical velocity ($J \dot{q}$) for each Cartesian axis over the validation trajectory. Both lines should overlap perfectly.

**Discussion:** The maximum error of ~$7.5 \times 10^{-5}$ m/s confirms the Jacobian is mathematically consistent with the Direct Kinematics model.

### 4.2 Singularity Rank Analysis

**Results Table:**
| Configuration | $q$ values | Jacobian Rank | Loss |
|---|---|---|---|
| Generic Pose | $[0.1, 0.5, …]$ | 6 | None |
| Elbow Stretched ($q_4=0$) | $[0, \pi/4, 0, 0, …]$ | 5 | **Task-Space DOF** |
| Shoulder ($q_2=0$) | $[0, 0, \pi/4, …]$ | 6 | Redundancy only |
| Wrist ($q_6=0$) | $[0, \pi/4, 0, \pi/2, 0, 0, 0]$ | 6 | Redundancy only |

**Discussion:** The key finding here is the distinction between two fundamentally different types of singularity in a 7-DOF robot:
- The **Elbow Singularity** causes a true loss of controllability (the end-effector cannot be pushed outward). This is dangerous in practice and must be avoided during trajectory planning.
- The **Shoulder/Wrist Singularities** only consume the robot's built-in redundancy. The end-effector can still move in all 6 directions — the robot simply loses the ability to rearrange its internal configuration. For a medical robot, this is far less dangerous than the elbow case.

---

## 5. Closed-Loop Inverse Kinematics (CLIK) *(~2 pages)*

**Implementation:** The CLIK controller was implemented in `clik_controller.m` and wired into a Simulink model (`Kuka_CLIK_Model.slx`) generated automatically by `generate_clik_model.m`. Two control methods were implemented and are selectable at runtime:
- **Method 1 — Damped Pseudoinverse:** $J^{\dagger} = J^T (JJ^T + \lambda^2 I)^{-1}$. The damping factor $\lambda = 10^{-3}$ regularises the inversion near singularities, preventing runaway joint velocities at the cost of a small steady-state residual error.
- **Method 2 — Jacobian Transpose:** $\dot{q} = J^T \dot{x}_{ref}$. Requires no matrix inversion and is computationally cheaper, but convergence depends strongly on gain tuning and it provides no singularity protection.

In both methods, redundancy is managed via **null-space stabilisation**: $\dot{q}_0 = -K_n (q - q_{rest})$ is projected onto the null-space of the Jacobian using $(I - J^{\dagger} J)$, pulling the arm towards its resting posture without disturbing the end-effector path. The orientation error is computed using the axis-angle decomposition of the rotation error matrix $R_{err} = R_d R_e^T$, which remains numerically stable at all orientations (including $\theta = 0$ and $\theta = \pi$).

*Screenshot of `Kuka_CLIK_Model.slx` block diagram here.*

### 5.1 Method Comparison: Damped Pseudoinverse vs. Jacobian Transpose

**Graph 1 — Convergence Speed Comparison:**
Two subplots (position error and orientation error) each showing both methods on the same axes over time (0–6 s).

**Graph 2 — Joint Trajectories:**
Two subplots showing $q(t)$ for all 7 joints under each method side-by-side. This visually shows how differently the two methods exploit the null-space.

**Results Table:**
| Metric | Damped Pseudoinverse | Jacobian Transpose |
|---|---|---|
| Final position error (mm) | … | … |
| Final orientation error | … | … |
| Convergence time (approx.) | … s | … s |
| Oscillation/overshoot? | … | … |

**Discussion:**
- Which method converges faster? Which is smoother?
- The Damped Pseudoinverse ($\lambda = 10^{-3}$) guarantees a unique, stable solution even near singularities, at the cost of a small residual error.
- The Jacobian Transpose is simpler but may oscillate or converge more slowly if the gain $K$ is not well-tuned.

### 5.2 Null-Space Stabilisation

**Implementation:** Two runs of `validate_clik.m` were performed with identical gains — one with $K_n = 0.15 I_7$ (enabled) and one with $K_n = 0$ (disabled). Both should converge the end-effector to the same target, but the joint-space trajectories will differ significantly.

**Graph 3:** Show a simulation run *without* null-space stabilisation ($K_n = 0$) and *with* it ($K_n = 0.15$) on the same plot. Compare the joint trajectories — without it, the redundant joints should drift or oscillate.

**Discussion:** What does this confirm about the importance of null-space control for redundant manipulators? For a robot used in surgery, an arm that wanders into a physically unstable posture mid-task would be unacceptable.

### 5.3 CLIK vs. Analytical IK — Comparison

**Results Table:**
| Metric | Analytical IK (Step 3) | CLIK (Step 5) |
|---|---|---|
| Position error (mm) | … | … |
| Orientation error | … | … |
| Handles trajectory? | ❌ (point-to-point only) | ✅ (continuous) |
| Robust to singularities? | ❌ | ✅ (damping) |
| Real-time capable? | ✅ (no iteration) | ✅ (small $dt$) |

**Discussion:** Analytical IK delivers perfect accuracy for isolated target poses but is inflexible. CLIK introduces a small residual error (due to damping) but is the only practical solution for continuous trajectory following, redundancy management, and robust behaviour near singularities.

---

## 6. Conclusions *(~0.5 pages)*

**Draw 3–4 specific, evidence-based conclusions:**

1. The D-H model and Direct Kinematics are **mathematically exact**, confirmed by 0 mm error across all configurations.
2. The Analytical IK is **accurate for reachable targets** but fails at singular configurations and is unsuitable for real-time trajectory tracking due to its geometric assumptions.
3. The Jacobian is **consistent with the DK** (max error ~$7.5 \times 10^{-5}$ m/s) and correctly identifies the elbow as the only configuration causing a true loss of task-space mobility.
4. CLIK with the **Damped Pseudoinverse and null-space stabilisation** is the most robust control strategy for the LBR MED — it maintains stability near singularities and prevents uncontrolled posture drift, which are critical requirements for a medical robot.

