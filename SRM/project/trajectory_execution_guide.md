# Beginner's Guide: Surgical RCM Trajectory Execution

This guide explains how the trajectory execution for the **KUKA LBR MED** with a Remote Center of Motion (RCM) constraint is implemented in the codebase. It details where each function resides, the underlying mathematical principles, and the specific lines of code that make it work.

---

## 1. File Map: What Code Runs Where?

All files related to the Phase II trajectory execution reside in the directory `07_phase_II`:

1. **Initial Posture Solver**: [find_initial_configuration.m](find_initial_configuration.m)
   - *Purpose*: Calculates the initial joint angles ($q_0$) that position the robot's end-effector flange **above** the patient and point the needle tool straight down through the trocar.
2. **Trajectory Planner**: [trajectory_planner.m](trajectory_planner.m)
   - *Purpose*: Generates a smooth, continuous path connecting the target points inside the patient using a cubic timing law.
3. **RCM CLIK Controller**: [rcm_clik_controller.m](rcm_clik_controller.m)
   - *Purpose*: Solves the robot velocities dynamically, prioritizing the trocar constraint above the target-tracking task.
4. **Simulink Model Builder**: [build_surgical_simulink.m](build_surgical_simulink.m)
   - *Purpose*: Automatically generates the Simulink model block diagram (`Surgical_RCM_Sim.slx`), configuring the initial joint values, solver settings, and signal lines.
5. **Simulation Validation & Plotter**: [validate_rcm_simulation.m](validate_rcm_simulation.m)
   - *Purpose*: Runs the Simulink simulation, extracts variables from the workspace, validates constraints, and saves visual plots.

---

## 2. Step-by-Step Code Walkthrough

### Step A: Starting Above the Trocar (Guaranteeing Correct Orientation)
Before the simulation starts, the robot must be in a position where the end-effector flange is **above** the patient (trocar height $z_{trocar} = 0.4\text{ m}$) and pointing the needle downward.

In [find_initial_configuration.m](find_initial_configuration.m), we enforce this:
```matlab
% Lines 13-15: Define coordinates
c_r = [-0.5; 0.0; 0.4];  % Trocar position (z = 0.4)
m0 = [-0.5; 0.0; 0.3];   % Start target (z = 0.3, inside patient)
L = 0.15;                % 15 cm needle tool

% Line 21: Flange must be 15 cm directly above the tooltip
p_d = m0 - L * dir_down;  % Computes p_d = [-0.5; 0.0; 0.45] (above trocar!)

% Lines 24-26: Force the tool Z-axis to point straight down (third column is [0; 0; -1])
R_d = [1  0  0;
       0 -1  0;
       0  0 -1];
```
This target pose is fed into the analytical inverse kinematics solver [inverse_kinematics.m](../03_inverse_kinematics/inverse_kinematics.m), returning the starting joint angles:
`q_0 = [0; 0.4085; 3.1416; 0.6128; -3.1416; 2.1203; 3.1416]`

---

### Step B: The Smooth Trajectory Planner
To move the tooltip between the 3D targets ($m_0 \to m_1 \to m_2 \to m_3$), we divide the simulation into 5-second segments. We use a **cubic timing law** to transition smoothly. 

In [trajectory_planner.m](trajectory_planner.m), the interpolation is handled as follows:
```matlab
% Lines 27-34: Compute normalized scaling factor 's' (0 to 1) and its derivative
if tau <= 0
    s = 0; s_dot = 0;
elseif tau >= T_seg
    s = 1; s_dot = 0;
else
    % Cubic polynomial timing law
    s = 3*(tau/T_seg)^2 - 2*(tau/T_seg)^3;
    s_dot = (6*tau/(T_seg^2)) - (6*tau^2/(T_seg^3));
end

% Lines 36-37: Linear interpolation between segment start and end points
p_d = p_start + (p_end - p_start) * s;
v_d = (p_end - p_start) * s_dot;
```
This timing law ensures that the velocity starts at zero, peaks in the middle of the segment, and slows down to zero exactly as it reaches each target, avoiding sharp, jerky joint motions.

---

### Step C: Enforcing the Trocar Constraint (RCM)
The core requirement of minimally invasive surgery is that the needle shaft cannot move laterally at the incision (trocar). However, the needle **is** allowed to slide forward/backward through the trocar (insertion depth) and rotate around it.

In [rcm_clik_controller.m](rcm_clik_controller.m), we enforce this mathematically:
1. **Find Insertion Depth ($\lambda$):** We project the vector from the flange ($p_e$) to the trocar ($c_r$) along the tool axis ($z_e$):
   ```matlab
   % Line 36
   lambda = (c_r - p_e)' * z_e;
   ```
2. **Compute Closest Point ($p_c$):** The point on the physical needle shaft closest to the trocar:
   ```matlab
   % Line 37
   p_c = p_e + lambda * z_e;
   ```
3. **Calculate Constraint Error ($e_{rcm}$):** The lateral displacement from the trocar center:
   ```matlab
   % Line 38
   e_rcm = c_r - p_c;
   ```
4. **Project Orthogonal to Tool Axis:** To let the needle slide freely along its axis (meaning joint movements that insert the tool do not violate the constraint), we project the RCM Jacobian onto the plane orthogonal to $z_e$:
   ```matlab
   % Lines 45-46
   Proj_ortho = eye(3) - z_e * z_e';
   J_rcm = Proj_ortho * J_pc;
   ```

---

### Step D: Task-Priority Control (CLIK)
Because violating the patient's body wall is dangerous, the RCM constraint is a **hard task** (Priority 1), and tracking the trajectory is a **secondary task** (Priority 2). We solve this using a task-priority projection.

In [rcm_clik_controller.m](rcm_clik_controller.m), this is computed:
```matlab
% Lines 40-41 (Task 1 - RCM, highest priority)
J_rcm_pinv = pinv(J_rcm);
q_dot_1 = J_rcm_pinv * (K_rcm * e_rcm);

% Lines 44-49 (Task 2 - Tool position tracking, projected in the null-space of Task 1)
e_tool = p_d - p_tool;
v_tool_1 = J_tool * q_dot_1;
v_tool_req = v_d + K_tool * e_tool - v_tool_1;

% Null-space projector of Task 1: (I - J_rcm_pinv * J_rcm)
N_1 = eye(7) - J_rcm_pinv * J_rcm;
J_tool_N = J_tool * N_1;

% Line 53-57: Damped Least Squares to get q_dot_2
J_tool_N_pinv = J_tool_N' / (J_tool_N * J_tool_N' + lambda_damp^2 * eye(3));
q_dot_2 = J_tool_N_pinv * v_tool_req;

% Combined command
q_dot = q_dot_1 + q_dot_2;
```
* **How it works**: $q_{dot1}$ only moves the joints to keep the needle in the trocar. $q_{dot2}$ moves the tooltip toward the target targets, but its projection through the null-space matrix $N_1$ filters out any joint motions that would violate the trocar constraint.
