# KUKA LBR MED Project

Project files for SRMan Phase I.

## Folders

| Folder                   | What is inside                                         |
| ------------------------ | ------------------------------------------------------ |
| `01_dh_model/`           | D-H table and `KukaLBR.m`                              |
| `02_direct_kinematics/`  | direct kinematics generator and notes                  |
| `03_inverse_kinematics/` | inverse kinematics code and notes                      |
| `04_jacobian/`           | Jacobian generator and notes                           |
| `05_clik/`               | CLIK controller, model generator, experiments, metrics |
| `06_visualization/`      | 3D visualization scripts                               |
| `validations/`           | validation scripts for steps 2 to 5                    |
| `generated/`             | generated MATLAB functions                             |
| `simulink/`              | Simulink models and libraries                          |
| `figures/`               | figures used in the report                             |
| `slprj/`                 | Simulink cache/build files                             |

Main files in this folder:
- `startup.m`
- `generate_report_figures.m`

## How to run

Open MATLAB in `SRM/project` and run:

```matlab
startup
```

Run the validations:

```matlab
run('validations/validate_direct_kinematics.m')
run('validations/validate_inverse_kinematics.m')
run('validations/validate_jacobian.m')
run('validations/validate_clik.m')
```

Regenerate generated functions or models:

```matlab
run('02_direct_kinematics/generate_kuka_library.m')
run('04_jacobian/generate_jacobian_library.m')
run('05_clik/generate_clik_model.m')
```

Regenerate figures:

```matlab
run('generate_report_figures.m')
run('05_clik/step5_clik_experiments.m')
```

## Notes

- Run `04_jacobian/generate_jacobian_library.m` if `jacobian_kuka.m` or `kuka_direct_kinematics.m` is missing.
- The main Simulink files are in `simulink/`.
- The report figures are saved in `figures/`.
