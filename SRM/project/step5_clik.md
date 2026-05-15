# Step 5 & 3D Visualisation — How to Run

## Prerequisites
Before running anything in this section, make sure Steps 1–4 have been completed.
If you haven't already, run the Jacobian generator once:

```matlab
cd('SRM/project')
run('generate_jacobian_library.m')
```

This creates `jacobian_kuka.m` and `kuka_direct_kinematics.m` which every script below depends on.

---

## Step 5: CLIK Simulation

### 1. Generate the Simulink Model
This creates `Kuka_CLIK_Model.slx` in the project folder.

```matlab
run('generate_clik_model.m')
```

### 2. Validate the Control Law (MATLAB)
Runs the CLIK integration loop in pure MATLAB (no Simulink needed).
Prints position/orientation errors and shows two convergence plots.

```matlab
run('validate_clik.m')
```

**Expected output:**
```
CLIK final pose check:
  position error [mm] = < 1.0
  orientation error   = < 0.01
  SUCCESS: CLIK converged to the desired pose.
```

### 3. Run in Simulink (optional)
Open the generated model and run it for 6 seconds:

```matlab
open_system('Kuka_CLIK_Model')
sim('Kuka_CLIK_Model')
```

The scopes inside will show joint angles (`q_scope`) and the task-space error (`error_scope`) over time.

---

## 3D Visualisation

> **Note:** The professor's `RobotX_sim3d` package requires the Simulation 3D
> toolbox (`sim3dlib`) which may cause MATLAB to crash depending on your GPU
> drivers. The visualiser below uses only core MATLAB and is fully stable.

### Run the 3D Visualiser

First run the CLIK validation to generate the trajectory data, then launch the visualiser:

```matlab
run('validate_clik.m')        % Step 1: run the simulation (saves q_hist)
run('visualize_robot_3d.m')   % Step 2: animate the robot
```

The visualiser will open a dark 3D figure showing:
- The robot arm moving from its initial pose to the CLIK target
- A **gold trail** tracing the path of the end-effector
- A live timer in the corner matching the simulation time

Close the figure window at any time to stop the animation.

---

## Run Everything at Once

To reproduce the full Step 5 pipeline from scratch:

```matlab
cd('SRM/project')
run('generate_jacobian_library.m')   % Generate Jacobian (if not done already)
run('generate_clik_model.m')         % Build Simulink model
run('validate_clik.m')               % Run CLIK + print results + show plots
run('visualize_robot_3d.m')          % 3D animation of the trajectory
```

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `jacobian_kuka.m not found` | Run `generate_jacobian_library.m` first |
| `clik_controller.m not found` | Make sure you are `cd`'d into `SRM/project` |
| 3D figure is empty / no motion | Check `q_hist` exists in the workspace (`whos q_hist`) |
| MATLAB crashes on Sim3D | Use `visualize_robot_3d.m` instead of `generate_kuka_visual.m` |
