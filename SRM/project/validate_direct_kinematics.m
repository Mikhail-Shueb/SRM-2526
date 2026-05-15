%% validate_direct_kinematics.m
%  Step 2 - Validation of the Direct Kinematics model
%  ---------------------------------------------------
%  This script validates the KukaLBR() D-H model by testing it at
%  known configurations where we can predict the result by hand.
%
%  HOW TO RUN:
%    - Run generate_kuka_library.m FIRST to build the symbolic model.
%    - Then run this script: run('validate_direct_kinematics.m')

%% ---- Setup ----
projectPath = fileparts(mfilename('fullpath'));
toolboxPath = fullfile(fileparts(projectPath), 'toolbox');
addpath(projectPath);
addpath(toolboxPath);

disp('=== KUKA LBR MED - Direct Kinematics Validation ===');

% Get the symbolic transformation matrix
Robot = KukaLBR();
disp('Computing T symbolically (may take a few minutes)...');
T_sym = DKin(Robot);
disp('Done.');

%% ---- Helper: evaluate T at a given set of joint angles ----
% symvar(Robot) returns [q1 q2 q3 q4 q5 q6 q7] in order.
% We substitute numerical values using subs().
syms q1 q2 q3 q4 q5 q6 q7 real

%% ---- TEST 1: Home position (all joints = 0) ----
q_test1 = [0, 0, 0, 0, 0, 0, 0];
T1 = double(subs(T_sym, [q1,q2,q3,q4,q5,q6,q7], q_test1));
p1 = T1(1:3, 4);
p1_exp = [0; 0; 0.926];
if norm(p1 - p1_exp) < 1e-4; res1 = 'Correct'; else; res1 = 'Wrong'; end
disp(['1. Home position        -> ', res1]);

%% ---- TEST 2: Joint 2 at 90 degrees (shoulder bend) ----
q_test2 = [0, pi/2, 0, 0, 0, 0, 0];
T2 = double(subs(T_sym, [q1,q2,q3,q4,q5,q6,q7], q_test2));
p2 = T2(1:3, 4);
p2_exp = [-0.926; 0; 0];
if norm(p2 - p2_exp) < 1e-4; res2 = 'Correct'; else; res2 = 'Wrong'; end
disp(['2. Shoulder bend        -> ', res2]);

%% ---- TEST 3: Elbow Bend only (L-shape) ----
q_test3 = [0, 0, 0, -pi/2, 0, 0, 0];
T3 = double(subs(T_sym, [q1,q2,q3,q4,q5,q6,q7], q_test3));
p3 = T3(1:3, 4);
p3_exp = [-0.526; 0; 0.400];
if norm(p3 - p3_exp) < 1e-4; res3 = 'Correct'; else; res3 = 'Wrong'; end
disp(['3. L-shape bend         -> ', res3]);

%% ---- Summary ----
disp(' ');
disp('=== Validation complete! ===');
