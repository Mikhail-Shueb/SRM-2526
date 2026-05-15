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
q_fwd   = [0; 0; 0; 0; 0; 0; 0];
T_fwd   = fk(q_fwd);
R_goal  = T_fwd(1:3,1:3);
p_goal  = T_fwd(1:3,4);

q_inv   = inverse_kinematics(R_goal, p_goal, 0);
T_check = fk(q_inv);

err_p = norm(T_check(1:3,4) - p_goal)*1000;
if err_p < 1; res1 = 'Correct'; else; res1 = 'Wrong'; end
disp(['1. Home position        -> ', res1]);

%% TEST 2 — Elbow bent (q2=pi/4, q4=pi/3)
q_fwd   = [0; pi/4; 0; pi/3; 0; pi/6; 0];
T_fwd   = fk(q_fwd);
R_goal  = T_fwd(1:3,1:3);
p_goal  = T_fwd(1:3,4);

q_inv   = inverse_kinematics(R_goal, p_goal, 0);
T_check = fk(q_inv);

err_p = norm(T_check(1:3,4) - p_goal)*1000;
err_o = norm(T_check(1:3,1:3) - R_goal,'fro');
if err_p < 1 && err_o < 0.01; res2 = 'Correct'; else; res2 = 'Wrong'; end
disp(['2. Elbow bent           -> ', res2]);

%% TEST 3 — Random configuration
rng(42);
q_fwd   = (rand(7,1) - 0.5) * pi/2;   % random angles in [-pi/4, pi/4]
T_fwd   = fk(q_fwd);
R_goal  = T_fwd(1:3,1:3);
p_goal  = T_fwd(1:3,4);

q_inv   = inverse_kinematics(R_goal, p_goal, 0);
T_check = fk(q_inv);

err_p = norm(T_check(1:3,4) - p_goal)*1000;
err_o = norm(T_check(1:3,1:3) - R_goal,'fro');
if err_p < 1 && err_o < 0.01; res3 = 'Correct'; else; res3 = 'Wrong'; end
disp(['3. Random configuration -> ', res3]);

disp(' ');
disp('=== Validation complete! ===');
