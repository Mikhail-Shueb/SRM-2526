# KUKA LBR MED Project Organization

This folder is organized from the project PDF into Phase I steps and validation checkpoints.

## Folder Map

| Folder | Purpose |
|---|---|
| `01_dh_model/` | Step 1: D-H model, reference-frame work, and `KukaLBR.m` |
| `02_direct_kinematics/` | Step 2: direct kinematics guide and library generator |
| `03_inverse_kinematics/` | Step 3: analytical inverse kinematics implementation and guide |
| `04_jacobian/` | Step 4: geometric Jacobian derivation and generator |
| `05_clik/` | Step 5: closed-loop inverse kinematics controller, model generator, experiments, and metrics |
| `06_visualization/` | 3D visualization helpers and Simulink visual connection scripts |
| `figures/` | Generated report figures and experiment plots |
| `validations/` | Validation scripts for Steps 2 through 5 |
| `generated/` | Generated MATLAB functions used by validation and control scripts |
| `simulink/` | Simulink libraries and models |
| `slprj/` | MATLAB/Simulink generated build/cache files |

Root files:
- `startup.m` adds the organized project folders and toolbox to the MATLAB path.
- `generate_report_figures.m` regenerates the static robot pose figures into `figures/`.

## Step Plan And Validations

### Step 1: D-H Model
Deliverables:
- Draw the joint/reference-frame diagram.
- Define the D-H table in `01_dh_model/KukaLBR.m`.

Validation:
- Check that the table has 7 revolute joints.
- Confirm link offsets match the KUKA LBR MED geometry used in the report.

### Step 2: Direct Kinematics
Deliverables:
- Generate the direct kinematics block with `02_direct_kinematics/generate_kuka_library.m`.
- Store the Simulink library in `simulink/Kuka_Lib.slx`.

Validation:
- Run `validations/validate_direct_kinematics.m`.
- Test simple robot poses whose end-effector position/orientation can be checked geometrically.

### Step 3: Analytical Inverse Kinematics
Deliverables:
- Implement one closed-form redundant IK solution in `03_inverse_kinematics/inverse_kinematics.m`.
- Identify singularity and reachability limits.

Validation:
- Run `validations/validate_inverse_kinematics.m`.
- Feed IK results back through direct kinematics and compare against the target pose.

### Step 4: Geometric Jacobian
Deliverables:
- Generate the symbolic geometric Jacobian with `04_jacobian/generate_jacobian_library.m`.
- Store generated functions in `generated/`.

Validation:
- Run `validations/validate_jacobian.m`.
- Compare numerical differentiation of direct kinematics with `J * qdot`.
- Check Jacobian rank at singular configurations.

### Step 5: CLIK With Null-Space Stabilization
Deliverables:
- Generate the CLIK Simulink model with `05_clik/generate_clik_model.m`.
- Implement controller logic in `05_clik/clik_controller.m`.
- Store experiment metrics in `05_clik/step5_clik_experiment_metrics.csv`.
- Store comparison plots in `figures/`.

Validation:
- Run `validations/validate_clik.m`.
- Compare CLIK results against analytical IK from Step 3.
- Compare joint behavior with null-space stabilization enabled and disabled.

## MATLAB Setup

Open MATLAB in this folder and run:

```matlab
startup
```

Then run scripts from the project root, for example:

```matlab
run('04_jacobian/generate_jacobian_library.m')
run('validations/validate_jacobian.m')
```

To regenerate the report figures:

```matlab
run('generate_report_figures.m')
run('05_clik/step5_clik_experiments.m')
```
