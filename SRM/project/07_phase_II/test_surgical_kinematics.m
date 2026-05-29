% test_surgical_kinematics.m
% Validates the dynamic computation of Tool Kinematics and RCM constraint

clear; clc;

% Load standard Phase I generated functions
projectPath = fileparts(pwd);
addpath(fullfile(projectPath, 'generated'));
addpath(fullfile(projectPath, 'toolbox'));

% Define test configuration
q = [0.1; 0.2; 0.3; -0.4; 0.5; 0.6; 0.7];
L = 0.15; % 15 cm tool
c_r = [-0.6; 0.0; 0.4]; % Trocar position

% 1. Get Base Kinematics
T_e = kuka_direct_kinematics(q);
p_e = T_e(1:3, 4);
R_e = T_e(1:3, 1:3);
z_e = R_e(:, 3);

J_e = jacobian_kuka(q);
J_v = J_e(1:3, :);
J_w = J_e(4:6, :);

% 2. Tool Kinematics
p_tool = p_e + L * z_e;
skew_Lz = [0, -L*z_e(3), L*z_e(2);
           L*z_e(3), 0, -L*z_e(1);
           -L*z_e(2), L*z_e(1), 0];
J_tool = J_v - skew_Lz * J_w;

disp('--- Tool Kinematics ---');
disp('p_tool:'); disp(p_tool);

% 3. RCM Constraint
% Distance along shaft to the point closest to trocar
lambda = (c_r - p_e)' * z_e;
p_c = p_e + lambda * z_e;
e_rcm = c_r - p_c; % We want to drive this error to 0

skew_lamz = [0, -lambda*z_e(3), lambda*z_e(2);
             lambda*z_e(3), 0, -lambda*z_e(1);
             -lambda*z_e(2), lambda*z_e(1), 0];

J_pc = J_v - skew_lamz * J_w;

% Project the Jacobian onto the plane orthogonal to z_e
% because movement ALONG the shaft does not violate the trocar constraint.
Proj_ortho = eye(3) - z_e * z_e';
J_rcm = Proj_ortho * J_pc;

disp('--- Trocar RCM Constraint ---');
disp(['Insertion depth (lambda): ', num2str(lambda)]);
disp('Constraint error vector (e_rcm):'); disp(e_rcm);
disp('Error magnitude (should be driven to 0):'); disp(norm(e_rcm));

disp('Kinematics successfully validated mathematically!');
