%% validate_inverse_kinematics.m
%  Step 3 — Validate the Inverse Kinematics against Direct Kinematics
%  -------------------------------------------------------------------
%  Run generate_kuka_library.m FIRST to build the symbolic DK model.
%  Then run this script to check that IK + DK round-trip is correct.

projectPath = fileparts(mfilename('fullpath'));
toolboxPath = fullfile(fileparts(projectPath), 'toolbox');
addpath(projectPath); addpath(toolboxPath);

disp('=== KUKA LBR MED — Inverse Kinematics Validation ===');
disp('Computing symbolic DK (may take a few minutes)...');
Robot  = KukaLBR();
T_sym  = DKin(Robot);
syms q1 q2 q3 q4 q5 q6 q7 real
disp('Done.');

%% Helper: forward kinematics at a numeric joint vector
fk = @(q_vec) double(subs(T_sym, [q1,q2,q3,q4,q5,q6,q7], q_vec'));

%% TEST 1 — Recover the home position (all joints = 0)
disp(' '); disp('--- TEST 1: Home position (all q = 0) ---');
q_fwd   = [0; 0; 0; 0; 0; 0; 0];
T_fwd   = fk(q_fwd);
R_goal  = T_fwd(1:3,1:3);
p_goal  = T_fwd(1:3,4);

q_inv   = inverse_kinematics(R_goal, p_goal, 0);
T_check = fk(q_inv);

disp('Original joints:');   disp(q_fwd');
disp('IK solution:');       disp(q_inv');
disp('Position error [mm]:');
disp(norm(T_check(1:3,4) - p_goal)*1000);

%% TEST 2 — Elbow bent (q2=pi/4, q4=pi/3)
disp(' '); disp('--- TEST 2: Elbow bent (q2=45 deg, q4=60 deg) ---');
q_fwd   = [0; pi/4; 0; pi/3; 0; pi/6; 0];
T_fwd   = fk(q_fwd);
R_goal  = T_fwd(1:3,1:3);
p_goal  = T_fwd(1:3,4);

q_inv   = inverse_kinematics(R_goal, p_goal, 0);
T_check = fk(q_inv);

disp('Position error [mm]:');
disp(norm(T_check(1:3,4) - p_goal)*1000);
disp('Orientation error (Frobenius norm of R difference):');
disp(norm(T_check(1:3,1:3) - R_goal,'fro'));

%% TEST 3 — Random configuration
disp(' '); disp('--- TEST 3: Random joint configuration ---');
rng(42);
q_fwd   = (rand(7,1) - 0.5) * pi/2;   % random angles in [-pi/4, pi/4]
T_fwd   = fk(q_fwd);
R_goal  = T_fwd(1:3,1:3);
p_goal  = T_fwd(1:3,4);

q_inv   = inverse_kinematics(R_goal, p_goal, 0);
T_check = fk(q_inv);

disp('Position error [mm]:');
disp(norm(T_check(1:3,4) - p_goal)*1000);
disp('Orientation error:');
disp(norm(T_check(1:3,1:3) - R_goal,'fro'));

disp(' ');
disp('=== Validation complete! ===');
disp('Errors < 1 mm and < 0.01 mean the IK is correct.');
