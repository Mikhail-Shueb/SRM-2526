# Step 5: CLIK

## How to Run

```matlab
cd('SRM/project')
run('generate_jacobian_library.m')
run('generate_clik_model.m')
run('validate_clik.m')
```

Once you run everything you can open the simulink model:

```matlab
open_system('Kuka_CLIK_Model')
```
