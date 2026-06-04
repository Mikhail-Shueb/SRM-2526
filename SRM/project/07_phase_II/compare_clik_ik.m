% compare_clik_ik.m
% Compares CLIK and Analytical IK joint trajectories and errors for Phase II

clear; clc; close all;

% Set up paths
projectPath = fileparts(pwd);
workspacePath = fileparts(projectPath);
addpath(fullfile(projectPath, 'generated'));
addpath(fullfile(workspacePath, 'toolbox'));
addpath(fullfile(projectPath, '03_inverse_kinematics'));
addpath(pwd);

disp('Running RCM simulation to get CLIK data...');
simOut = sim('Surgical_RCM_Sim', 'StopTime', '30.0');
t = simOut.tout;
q_clik = simOut.get('q_data');
if isa(q_clik, 'timeseries')
    q_clik = q_clik.Data;
end
q_clik = squeeze(q_clik);
if size(q_clik, 1) ~= length(t)
    q_clik = q_clik';
end

disp('Computing Analytical IK for the same trajectory...');
q_ik = zeros(length(t), 7);
c_r = [-0.5; 0.0; 0.4];
L = 0.15;
psi = 0; % Elbow parameter

for i = 1:length(t)
    [p_d, ~] = trajectory_planner(t(i));
    
    % Direction vector
    dir_down = (p_d - c_r) / norm(p_d - c_r);
    p_flange = p_d - L * dir_down;
    
    z_e = dir_down;
    if abs(z_e(1)) < 0.9
        x_temp = [1; 0; 0];
    else
        x_temp = [0; 1; 0];
    end
    y_e = cross(z_e, x_temp);
    y_e = y_e / norm(y_e);
    x_e = cross(y_e, z_e);
    R_d = [x_e, y_e, z_e];
    
    try
        q_sol = inverse_kinematics(R_d, p_flange, psi);
        q_ik(i, :) = q_sol';
    catch
        if i > 1
            q_ik(i, :) = q_ik(i-1, :);
        else
            q_ik(i, :) = zeros(1, 7);
        end
    end
end

% Plot comparison of Joint 4 (Elbow) and Joint 6 (Wrist)
h_fig = figure('Name', 'CLIK vs Analytical IK Trajectory', 'Color', 'w', 'Position', [100, 100, 800, 600]);

subplot(2,1,1);
plot(t, q_clik(:, 4), 'b-', 'LineWidth', 2); hold on;
plot(t, q_ik(:, 4), 'r--', 'LineWidth', 2);
grid on;
title('Joint 4 (Elbow) Angle over Time');
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('CLIK (Smooth Integration)', 'Analytical IK (Open-Loop)');

subplot(2,1,2);
plot(t, q_clik(:, 6), 'b-', 'LineWidth', 2); hold on;
plot(t, q_ik(:, 6), 'r--', 'LineWidth', 2);
grid on;
title('Joint 6 (Wrist) Angle over Time');
xlabel('Time (s)');
ylabel('Angle (rad)');
legend('CLIK (Smooth Integration)', 'Analytical IK (Open-Loop)');

saveas(h_fig, 'clik_vs_ik_comparison.png');
disp('Comparison plot generated and saved as clik_vs_ik_comparison.png successfully.');
