% find_initial_configuration.m
% Finds a q_0 where the tooltip is at m0, and the tool orientation 
% points directly from the Trocar to m0, guaranteeing the flange is ABOVE.

clear; clc;

projectPath = fileparts(pwd);
workspacePath = fileparts(projectPath);
addpath(fullfile(projectPath, 'generated'));
addpath(fullfile(workspacePath, 'toolbox'));
addpath(fullfile(projectPath, '03_inverse_kinematics'));

c_r = [-0.5; 0.0; 0.4];
m0 = [-0.5; 0.0; 0.3];
L = 0.15;

% The vector pointing from the Trocar down to the target (direction of tool)
dir_down = (m0 - c_r) / norm(m0 - c_r); % should be [0; 0; -1]

% Desired flange position (above the trocar)
p_d = m0 - L * dir_down; % [-0.5; 0.0; 0.45]

% Desired rotation matrix where the 3rd column is z_e = dir_down
R_d = [1 0 0;
       0 -1 0;
       0 0 -1];

psi = 0; % elbow parameter

% Analytical Inverse Kinematics
q = inverse_kinematics(R_d, p_d, psi);

% Verify
T_e = kuka_direct_kinematics(q);
p_e_val = T_e(1:3, 4);
z_e_val = T_e(1:3, 3);
p_tool_val = p_e_val + L * z_e_val;

disp('Verification of Kinematics:');
disp(['Desired p_e: ', mat2str(p_d')]);
disp(['Actual p_e:  ', mat2str(p_e_val', 4)]);
disp(['Desired z_e: ', mat2str(dir_down')]);
disp(['Actual z_e:  ', mat2str(z_e_val', 4)]);
disp(['Desired tooltip: ', mat2str(m0')]);
disp(['Actual tooltip:  ', mat2str(p_tool_val', 4)]);

if p_e_val(3) > c_r(3)
    disp('SUCCESS: Flange is ABOVE the trocar.');
else
    disp('FAILURE: Flange is below the trocar.');
end

disp('Perfect Initial Joint Configuration (q_0):');
disp(mat2str(q, 5));
