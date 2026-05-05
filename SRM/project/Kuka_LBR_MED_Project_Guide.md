# KUKA LBR MED Project Guide: Step-by-Step

Welcome to your KUKA LBR MED Kinematics project! This guide will walk you through the instructions provided in your project PDF and show you exactly how to approach each step using the provided **Robotics Symbolic Matlab Toolbox**. 

---

## Phase I: Kinematics

### Step 1: Denavit-Hartenberg (D-H) Model
**Goal:** Create the mathematical skeleton of the robot.
**What to do:**
1. **Pen and Paper:** Look at the drawing of the KUKA LBR MED robot. It has 7 rotational joints (it's a redundant 7-DOF robot).
2. **Assign Frames:** Draw the $Z$ axis along each joint's rotation axis. Draw the $X$ and $Y$ axes following the right-hand rule.
3. **D-H Table:** Fill out the D-H parameter table for each of the 7 joints. The columns are usually:
   - $d$ (link offset)
   - $\theta$ (joint angle - denoted as $v$ in the toolbox)
   - $a$ (link length)
   - $\alpha$ (link twist)
4. **Matlab Code:** Create a new function file (e.g., `KukaLBR.m`) based on `RobotX.m` from the toolbox.
   ```matlab
   function Robot = KukaLBR()
       % Define symbolic variables for the 7 joints
       syms q1 q2 q3 q4 q5 q6 q7 real
       
       % Robot = [d  v  a  alpha  offset]
       Robot = [ 
           d1   q1   0   pi/2    0;
           0    q2   0  -pi/2    0;
           d3   q3   0   pi/2    0;
           % ... Fill in the rest based on your D-H table
       ];
   end
   ```

### Step 2: Direct Kinematics Simulink Model
**Goal:** Find out where the robot's end-effector is, given the joint angles.
**What to do:**
1. Open a new Matlab script (let's call it `generate_kuka_blocks.m`).
2. Use the `DKin` function from the toolbox to calculate the Transformation Matrix ($T$).
   ```matlab
   % 1. Create a new Simulink Library
   new_system('Kuka_Lib', 'Library');
   open_system('Kuka_Lib');
   
   % 2. Calculate Direct Kinematics
   [Kuka_T] = DKin(KukaLBR()); 
   
   % 3. Extract Rotation Matrix (R) and Position Vector (p)
   Kuka_R = Kuka_T(1:3, 1:3);
   Kuka_p = Kuka_T(1:3, 4);
   
   % 4. Generate Simulink Blocks
   matlabFunctionBlock('Kuka_Lib/Direct_Kinematics', Kuka_R, Kuka_p);
   
   save_system('Kuka_Lib');
   ```
3. **Validation:** Open a normal Simulink model, drag your new `Direct_Kinematics` block from `Kuka_Lib`, and feed it simple angles (like all zeros, or $90^\circ$ at specific joints). Check if the output $p$ (position) matches what you would expect geometrically.

### Step 3: Analytical Inverse Kinematics
**Goal:** Find the joint angles required to reach a specific position and orientation.
**What to do:**
1. **Math Work:** You need a closed-form solution. Since the KUKA LBR is redundant (7 joints for 6 degrees of freedom), there are infinite solutions. You will likely need to fix one parameter (often the "arm angle") to solve it. Use the **Kinematic Decoupling** approach (separating the position of the wrist from the orientation of the end-effector).
2. **Find Singularities:** Identify what joint configurations cause the math to fail (e.g., dividing by zero or arm fully stretched).
3. **Simulink Model:** Once you have the equations (e.g., $q_1 = f(x,y,z)$, etc.), you can write a Matlab function block in Simulink that takes the desired position/orientation as input and outputs the 7 joint angles $q$.
4. **Validation:** Connect the output of your Inverse Kinematics block into the input of your Direct Kinematics block. The final output should perfectly match your original input!

### Step 4: Geometric Jacobian
**Goal:** Calculate the relationship between joint velocities and end-effector velocities.
**What to do:**
1. You can calculate the Jacobian matrix ($J$) symbolically in Matlab. The Jacobian is a $6 \times 7$ matrix for this robot.
2. In your `generate_kuka_blocks.m` script, add:
   ```matlab
   % Assuming you calculate J using formulas from your slides:
   % J = [ ... ] 
   
   matlabFunctionBlock('Kuka_Lib/Jacobian', J);
   ```
3. **Validation:** Numerically differentiate your Direct Kinematics results over time ($\frac{\Delta p}{\Delta t}$) in Simulink and compare it to $J \cdot \dot{q}$.
4. **Singularities check:** Find the rank of $J$ using `rank(J)`. At singular positions (which you found in Step 3), the rank of $J$ will drop below 6.

### Step 5: Closed Loop Inverse Kinematics (CLIK)
**Goal:** Build a robust control loop to calculate inverse kinematics dynamically over time, utilizing the robot's redundancy.
**What to do:**
1. Create a new Simulink Model.
2. The core CLIK equation you need to implement using Simulink blocks (Sum, Gain, Matrix Multiply, Integrator) is:
   $$\dot{q} = J^{\dagger} (\dot{x}_d + K(x_d - x)) + (I - J^{\dagger}J)\dot{q}_0$$
   *Where:*
   - $J^{\dagger}$ is the pseudo-inverse of the Jacobian.
   - $x_d$ is the desired trajectory, $x$ is the current position (from Direct Kinematics).
   - $K$ is a proportional gain matrix.
   - $(I - J^{\dagger}J)\dot{q}_0$ is the **null-space projection**. This is what "stabilizes the null space". Because it's redundant, you can define a secondary objective $\dot{q}_0$ (like keeping joints close to their center limits) without affecting the main task!
3. Feed the resulting $\dot{q}$ into an Integrator block to get $q$, which then feeds back into your Direct Kinematics block to close the loop.
4. **Validation:** Compare the joint trajectories generated by this CLIK model with the results from your analytical solution in Step 3 for the same path.
