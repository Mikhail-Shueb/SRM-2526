% validate_rcm_simulation.m
% Runs the Surgical RCM Simulink model and plots validation metrics

clear; clc; close all;

% Set up paths
projectPath = fileparts(pwd);
workspacePath = fileparts(projectPath);
addpath(fullfile(projectPath, 'generated'));
addpath(fullfile(workspacePath, 'toolbox'));
addpath(pwd);

disp('Running Surgical RCM Simulink simulation...');
simOut = sim('Surgical_RCM_Sim', 'StopTime', '20.0');
disp('Simulation complete!');

% Extract time and joint data
t = simOut.tout;
q_data = simOut.get('q_data');

% If q_data is a timeseries (depending on MATLAB version), extract the data
if isa(q_data, 'timeseries')
    q_data = q_data.Data;
end

% Remove singleton dimensions (Simulink array output for vectors is often 7x1xN)
q_data = squeeze(q_data);

% Ensure q_data is Nx7 (time x joints)
if size(q_data, 1) ~= length(t) && size(q_data, 2) == length(t)
    q_data = q_data';
end

if size(q_data, 1) ~= length(t)
    error('q_data dimension mismatch with time vector t.');
end

% Parameters
L = 0.15;
c_r = [-0.5; 0.0; 0.4];
targets = [
    -0.5, 0.0, 0.3;   % m0 (Start)
    -0.5, 0.08, 0.28; % m1
    -0.5, -0.08, 0.28;% m2
    -0.55, 0.0, 0.30  % m3
];

% Pre-allocate tracking arrays
p_e_all = zeros(length(t), 3);
p_tool_all = zeros(length(t), 3);
e_rcm_mag = zeros(length(t), 1);

% Evaluate kinematics over time
for i = 1:length(t)
    q = q_data(i, :)';
    T_e = kuka_direct_kinematics(q);
    p_e = T_e(1:3, 4);
    z_e = T_e(1:3, 3);
    
    p_tool = p_e + L * z_e;
    p_e_all(i, :) = p_e';
    p_tool_all(i, :) = p_tool';
    
    lambda = (c_r - p_e)' * z_e;
    p_c = p_e + lambda * z_e;
    e_rcm_mag(i) = norm(c_r - p_c);
end

% Plot 1: RCM Error
h_fig1 = figure('Name', 'RCM Constraint Error', 'Color', 'w');
plot(t, e_rcm_mag * 1000, 'r', 'LineWidth', 2); % Convert to mm
grid on;
title('Trocar Constraint Error over Time');
xlabel('Time (s)');
ylabel('Distance from Trocar Center (mm)');

% Plot 2: 3D Trajectory
h_fig2 = figure('Name', 'Surgical Trajectory', 'Color', 'w');
hold on; grid on; axis equal; view(3);
title('Tooltip Trajectory inside the Body');
xlabel('X'); ylabel('Y'); zlabel('Z');

% Plot the path
plot3(p_tool_all(:,1), p_tool_all(:,2), p_tool_all(:,3), 'b-', 'LineWidth', 2);

% Plot Trocar
plot3(c_r(1), c_r(2), c_r(3), 'c^', 'MarkerSize', 10, 'MarkerFaceColor', 'c');

% Plot Targets
plot3(targets(:,1), targets(:,2), targets(:,3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');

% Plot Needle Shaft (Orange lines)
step_size = max(1, floor(length(t) / 20)); % Plot ~20 lines
h_shaft = [];
for i = 1:step_size:length(t)
    h = plot3([p_e_all(i,1), p_tool_all(i,1)], ...
              [p_e_all(i,2), p_tool_all(i,2)], ...
              [p_e_all(i,3), p_tool_all(i,3)], 'Color', [1 0.5 0], 'LineWidth', 1.5);
    if isempty(h_shaft)
        h_shaft = h; % Store one handle for the legend
    end
end

legend('Tooltip Path', 'Trocar', 'Targets', 'Needle Shaft');

% Save the plots as images
saveas(h_fig1, 'rcm_error.png');
saveas(h_fig2, 'trajectory_3d.png');
disp('Validation plots generated and saved as rcm_error.png and trajectory_3d.png successfully.');
