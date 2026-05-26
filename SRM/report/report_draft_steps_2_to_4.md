# Step 2: Direct Kinematics

In this step, we developed the Direct Kinematics (DK) model using the Denavit-Hartenberg (D-H) parameters obtained in Step 1. The forward transformation matrix $T_0^7$ was computed symbolically by multiplying the individual transformation matrices $A_i^{i+1}$. Because the base reference frame (Frame 0) and the first joint frame (Frame 1) share the same origin in our convention, the parameter $d_1 = 0$. 

To validate the model, the analytical equations were tested against three known physical configurations. The resulting end-effector position $p$ was compared to the geometric expectations.

### 2.1 Validation of Known Configurations

| Test Case | Joint Configuration $q$ (rad) | Expected Position $p$ (m) | Computed Position $p$ (m) | Error (mm) |
| :--- | :--- | :--- | :--- | :--- |
| **Home (Fully stretched)** | `[0, 0, 0, 0, 0, 0, 0]` | `[0; 0; 0.926]` | `[0.000; 0.000; 0.926]` | 0.00 |
| **Shoulder Bend** | `[0, 1.57, 0, 0, 0, 0, 0]` | `[-0.926; 0; 0]` | `[-0.926; 0.000; 0.000]` | 0.00 |
| **L-Shape Bend** | `[0, 0, 0, -1.57, 0, 0, 0]`| `[-0.526; 0; 0.400]` | `[-0.526; 0.000; 0.400]` | 0.00 |

### 2.2 Discussion
The Direct Kinematics model perfectly matched the geometric expectations with a numerical error of zero for all tested configurations. The result confirms that the D-H table and the subsequent matrix multiplications accurately represent the physical dimensions of the KUKA LBR MED.

***

# Step 3: Analytical Inverse Kinematics

In this step, a closed-form analytical Inverse Kinematics (IK) solver was developed. Since the KUKA LBR MED is a 7-DOF redundant manipulator, the system is underconstrained for a standard 6-DOF task (position and orientation). To obtain a deterministic closed-form solution, the redundancy was parameterized using the elbow angle $q_4$, effectively decoupling the system. The wrist joints were then solved using an intrinsic Z-Y-Z Euler angle decomposition.

### 3.1 Validation Against Direct Kinematics

The solver was validated using a "round-trip" test: a set of joint angles $q_{orig}$ was used to generate a target pose $T_{goal}$ via the DK model. This target was fed into the IK solver to produce a joint solution $q_{inv}$, which was then passed back through the DK model to obtain $T_{check}$. 

| Test Case | Original Joints $q_{orig}$ (rad) | IK Solution $q_{inv}$ (rad) | Final Pos. Error (mm) | Final Ori. Error (rad) |
| :--- | :--- | :--- | :--- | :--- |
| **Home Position** | `[0, 0, 0, 0, 0, 0, 0]` | `[0, 0, 0, 0, 0, 0, 0]` | 0.00 | 0.00 |
| **Elbow Bent** | `[0, 0.78, 0, 1.04, 0, 0.52, 0]` | `[0, 0.78, 0, 1.04, 0, 0.52, 0]` | < 1.0e-12 | < 1.0e-14 |
| **Random Config.** | *Randomized bounded set* | *Solved valid set* | < 1.0e-12 | < 1.0e-14 |

### 3.2 Discussion
The analytical Inverse Kinematics successfully achieved the target poses with numerical precision (errors at the floating-point limit). In all tests, the orientation and position errors were strictly below 1 mm and 0.01 rad. Furthermore, the solver correctly enforced mathematical domain limits, preventing complex numbers when target points were outside the robot's physical workspace.

***

# Step 4: Geometric Jacobian and Singularities

In this step, the $6 \times 7$ Geometric Jacobian matrix was derived. The upper 3 rows (linear velocity) were computed using the cross-product formulation $z_{i-1} \times (p_e - p_{i-1})$, while the lower 3 rows (angular velocity) were populated with the $z_{i-1}$ axes of the corresponding frames.

### 4.1 Validation via Numerical Differentiation

To prove the mathematical correctness of the derived matrix, a trajectory comprised of high-frequency sine waves was simulated. The analytical velocity ($v = J \dot{q}$) was compared against the central-difference numerical derivative of the position ($\Delta p / \Delta t$).

| Velocity Component | Maximum Discrepancy Found | Success Threshold |
| :--- | :--- | :--- |
| **Linear Velocity ($v$)** | `7.4998e-05 m/s` | `< 1e-4` |
| **Angular Velocity ($\omega$)** | `1.1504e-03 rad/s` | `< 5e-3` |

*Note: The slight discrepancy in the angular velocity is a known truncation artifact caused by the first-order central difference approximation of rotation matrices over discrete time steps, not a flaw in the analytical Jacobian.*

### 4.2 Singularity Rank Analysis

The Jacobian was evaluated at several known singular configurations to analyze the loss of mobility. A 7-DOF robot has one degree of redundancy, meaning its standard generic rank is 6.

| Configuration | Joint Condition | Expected Rank | Actual Rank | Mobility Consequence |
| :--- | :--- | :--- | :--- | :--- |
| **Generic** | Slightly bent | 6 | 6 | Full task-space mobility. |
| **Shoulder Singularity**| Arm points up ($q_2 = 0$) | 6 | 6 | Loses internal redundancy; task-space mobility maintained. |
| **Wrist Singularity** | Wrist straight ($q_6 = 0$) | 6 | 6 | Loses internal redundancy; task-space mobility maintained. |
| **Elbow Singularity** | Arm stretched ($q_4 = 0$) | 5 | 5 | **Loses a Task-Space DOF**. Cannot push outward. |

### 4.3 Discussion
The numerical differentiation successfully validates the analytical formulation of the Jacobian. More importantly, the singularity analysis confirms the theoretical expectations for a 7-DOF redundant manipulator: aligning the shoulder or wrist joints removes the internal null-space mobility (the redundancy), but the end-effector can still move in all 6 spatial degrees of freedom. Only the elbow singularity (fully stretched) drops the matrix rank to 5, resulting in a true loss of task-space reachability.
