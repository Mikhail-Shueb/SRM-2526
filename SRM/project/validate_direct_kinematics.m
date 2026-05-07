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
% When all angles are zero, the robot is fully stretched upward.
% The end-effector should be directly above the shoulder.
% Since the shoulder is at z=0, the height is d3+d5+d7.
% Expected p = [0; 0; d3+d5+d7] = [0; 0; 0.400+0.400+0.126]
%                               = [0; 0; 0.926]  (in metres)
disp(' ');
disp('--- TEST 1: All joints at 0 degrees (Home / fully stretched) ---');
q_test1 = [0, 0, 0, 0, 0, 0, 0];
T1 = double(subs(T_sym, [q1,q2,q3,q4,q5,q6,q7], q_test1));
p1 = T1(1:3, 4);
R1 = T1(1:3, 1:3);
disp('End-effector position p [m]:');
disp(p1);
disp('Expected: [0; 0; 0.926]');
disp('Rotation matrix R:');
disp(R1);
disp('Expected: Identity matrix (no rotation at home)');

%% ---- TEST 2: Joint 2 at 90 degrees (shoulder bend) ----
% Joint 2 is the first "bending" joint.
% Due to the new twist alpha=pi/2 on joint 1, a positive rotation 
% on joint 2 bends the arm towards the negative X-axis.
% The arm above joint 2 has total length d3+d5+d7 = 0.926 m.
% Expected: p = [-0.926, 0, 0].
disp(' ');
disp('--- TEST 2: Joint 2 = 90 deg, rest = 0 ---');
q_test2 = [0, pi/2, 0, 0, 0, 0, 0];
T2 = double(subs(T_sym, [q1,q2,q3,q4,q5,q6,q7], q_test2));
p2 = T2(1:3, 4);
disp('End-effector position p [m]:');
disp(p2);
disp('Expected: [-0.926; 0; 0]');

%% ---- TEST 3: Joint 2 = 90 deg AND Joint 4 = -90 deg (L-shape) ----
% This simulates a classic "L" shape. The upper arm (d3=0.4m) bends
% along -X. Then joint 4 rotates the lower arm.
% Because Z3 is along Y0, a rotation of -90 deg around Y0 rotates
% the -X direction down to the -Z direction.
% Expected: p = [-0.400, 0, -0.526].
disp(' ');
disp('--- TEST 3: Joint 2 = 90 deg, Joint 4 = -90 deg, rest = 0 ---');
q_test3 = [0, pi/2, 0, -pi/2, 0, 0, 0];
T3 = double(subs(T_sym, [q1,q2,q3,q4,q5,q6,q7], q_test3));
p3 = T3(1:3, 4);
disp('End-effector position p [m]:');
disp(p3);
disp('Expected: [-0.400; 0; -0.526]');

%% ---- Summary ----
disp(' ');
disp('=== Validation complete! ===');
disp('If the positions match your expectations, the D-H model is correct.');
disp('You can add more test cases by copying a TEST block above.');
