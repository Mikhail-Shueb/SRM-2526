% Replays the planned trajectory with analytical IK and plots the errors

clear; clc; close all;

% Load of the generated kinematics and the Phase I IK solver.
projectPath = fileparts(pwd);
workspacePath = fileparts(projectPath);
addpath(fullfile(projectPath, 'generated'));
addpath(fullfile(workspacePath, 'toolbox'));
addpath(fullfile(projectPath, '03_inverse_kinematics'));
addpath(pwd);

disp('Loading time vector from RCM Sim...');
simOut = sim('Surgical_RCM_Sim', 'StopTime', '30.0');
t = simOut.tout;

L = 0.15;
c_r = [-0.5; 0.0; 0.4];
targets = [
    -0.5, 0.0, 0.3;   % m0 start
    -0.5, 0.08, 0.28; 
    -0.52, 0.04, 0.27;
    -0.46, -0.04, 0.32;
    -0.5, -0.08, 0.28;
    -0.55, 0.0, 0.30  % m5 end
];
% Store values to plot later
p_e_all_ik = zeros(length(t), 3);
p_tool_all_ik = zeros(length(t), 3);
p_d_all = zeros(length(t), 3);
e_rcm_mag_ik = zeros(length(t), 1);
e_tool_mag_ik = zeros(length(t), 1);

psi = 0; % elbow parameter for the redundant IK solution

disp('Solving analytical IK over time...');
for i = 1:length(t)
    [p_d_t, ~] = trajectory_planner(t(i));
    p_d_all(i, :) = p_d_t';
    
    % Tool direction from the trocar down to the target
    dir_down = (p_d_t - c_r) / norm(p_d_t - c_r);
    p_flange = p_d_t - L * dir_down;
    
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
        
        T_e = kuka_direct_kinematics(q_sol);
        p_e_true = T_e(1:3, 4);
        z_e_true = T_e(1:3, 3);
        p_tool_true = p_e_true + L * z_e_true;
        
        p_e_all_ik(i, :) = p_e_true';
        p_tool_all_ik(i, :) = p_tool_true';
        
        lambda = (c_r - p_e_true)' * z_e_true;
        p_c = p_e_true + lambda * z_e_true;
        e_rcm_mag_ik(i) = norm(c_r - p_c);
        
        e_tool_mag_ik(i) = norm(p_d_t - p_tool_true);
    catch
        % If IK fails at one sample, keep the previous valid point
        if i > 1
            p_e_all_ik(i, :) = p_e_all_ik(i-1, :);
            p_tool_all_ik(i, :) = p_tool_all_ik(i-1, :);
            e_rcm_mag_ik(i) = e_rcm_mag_ik(i-1);
            e_tool_mag_ik(i) = e_tool_mag_ik(i-1);
        end
    end
end

% Plot 1: RCM error, with the same dark style used in the report.
h_fig1 = figure('Name', 'IK RCM Error', 'Color', 'k');
plot(t, e_rcm_mag_ik * 1000, 'r', 'LineWidth', 2);
grid on;
set(gca, 'Color', 'k');
set(gca, 'XColor', [0.8 0.8 0.8]);
set(gca, 'YColor', [0.8 0.8 0.8]);
set(gca, 'GridColor', [0.4 0.4 0.4]);
set(gca, 'GridAlpha', 0.4);
title('Trocar Constraint Error over Time (Analytical IK)', 'Color', [0.8 0.8 0.8]);
xlabel('Time (s)', 'Color', [0.8 0.8 0.8]);
ylabel('Distance from Trocar Center (mm)', 'Color', [0.8 0.8 0.8]);
ylim([0 0.12]);
xlim([0 30]);
saveas(h_fig1, 'ik_rcm_error.png');

% Plot 2: 3D trajectory.
h_fig2 = figure('Name', 'IK Trajectory', 'Color', 'w');
hold on; grid on; axis equal; view(3);
set(gca, 'Color', 'k');
set(gca, 'XColor', [0.8 0.8 0.8]);
set(gca, 'YColor', [0.8 0.8 0.8]);
set(gca, 'ZColor', [0.8 0.8 0.8]);
set(gca, 'GridColor', [0.4 0.4 0.4]);
set(gca, 'GridAlpha', 0.4);
title('Tooltip Trajectory inside the Body (Analytical IK)', 'Color', [0.6 0.6 0.6]);
xlabel('X', 'Color', [0.6 0.6 0.6]);
ylabel('Y', 'Color', [0.6 0.6 0.6]);
zlabel('Z', 'Color', [0.6 0.6 0.6]);

plot3(p_tool_all_ik(:,1), p_tool_all_ik(:,2), p_tool_all_ik(:,3), 'b-', 'LineWidth', 2);

p_d_offset = p_d_all;
p_d_offset(:, 2) = p_d_offset(:, 2) + 0.003;
plot3(p_d_offset(:,1), p_d_offset(:,2), p_d_offset(:,3), 'r--', 'LineWidth', 1.5);

plot3(c_r(1), c_r(2), c_r(3), 'c^', 'MarkerSize', 10, 'MarkerFaceColor', 'c');
plot3(targets(:,1), targets(:,2), targets(:,3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');

step_size = max(1, floor(length(t) / 20));
for i = 1:step_size:length(t)
    plot3([p_e_all_ik(i,1), p_tool_all_ik(i,1)], ...
          [p_e_all_ik(i,2), p_tool_all_ik(i,2)], ...
          [p_e_all_ik(i,3), p_tool_all_ik(i,3)], 'Color', [1 0.5 0], 'LineWidth', 1.5);
end

lgd = legend('Executed Path (IK)', 'Planned Path (offset +3mm Y)', 'Trocar', 'Targets', 'Needle Shaft');
set(lgd, 'Color', 'k', 'TextColor', 'w', 'EdgeColor', [1 0.5 0]);
saveas(h_fig2, 'ik_trajectory_3d.png');

disp('IK validation plots generated and saved successfully.');
