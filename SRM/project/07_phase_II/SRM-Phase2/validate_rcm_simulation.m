% Validations

clear; clc; close all;

% Load the generated kinematics and the Phase II
projectPath = fileparts(pwd);
workspacePath = fileparts(projectPath);
addpath(fullfile(projectPath, 'generated'));
addpath(fullfile(workspacePath, 'toolbox'));
addpath(pwd);

disp('Running Surgical RCM Simulink simulation...');
simOut = sim('Surgical_RCM_Sim', 'StopTime', '30.0');
disp('Simulation complete!');

t = simOut.tout;
q_data = simOut.get('q_data');

if isa(q_data, 'timeseries')
    q_data = q_data.Data;
end

q_data = squeeze(q_data); % 7x1xN

% One row per time step
if size(q_data, 1) ~= length(t) && size(q_data, 2) == length(t)
    q_data = q_data';
end

if size(q_data, 1) ~= length(t)
    error('q_data dimension mismatch with time vector t.');
end

% Simulation parameters.
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

% Store pose and error values
p_e_all = zeros(length(t), 3);
p_tool_all = zeros(length(t), 3);
p_d_all = zeros(length(t), 3);
e_rcm_mag = zeros(length(t), 1);
e_tool_mag = zeros(length(t), 1);

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
    
    % Tooltip tracking error against the planned path.
    [p_d_t, ~] = trajectory_planner(t(i));
    p_d_all(i, :) = p_d_t';
    e_tool_mag(i) = norm(p_d_t - p_tool);
end

% Plot 1: trocar/RCM error
h_fig1 = figure('Name', 'RCM Constraint Error', 'Color', 'w');
plot(t, e_rcm_mag * 1000, 'r', 'LineWidth', 2); % convert to mm
grid on;
title('Trocar Constraint Error over Time');
xlabel('Time (s)');
ylabel('Distance from Trocar Center (mm)');

% Plot 2: 3D tooltip trajectory
h_fig2 = figure('Name', 'Surgical Trajectory', 'Color', 'w');
hold on; grid on; axis equal; view(3);
title('Tooltip Trajectory inside the Body');
xlabel('X'); ylabel('Y'); zlabel('Z');

% Executed path from the simulation
plot3(p_tool_all(:,1), p_tool_all(:,2), p_tool_all(:,3), 'b-', 'LineWidth', 2);

% Shift the planned path slightly so both curves are visible
p_d_offset = p_d_all;
p_d_offset(:, 2) = p_d_offset(:, 2) + 0.003;
plot3(p_d_offset(:,1), p_d_offset(:,2), p_d_offset(:,3), 'r--', 'LineWidth', 1.5);

% Trocar marker
plot3(c_r(1), c_r(2), c_r(3), 'c^', 'MarkerSize', 10, 'MarkerFaceColor', 'c');

% Target markers
plot3(targets(:,1), targets(:,2), targets(:,3), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');

% Draw a few needle shafts along the path
step_size = max(1, floor(length(t) / 20)); % about 20 line
h_shaft = [];
for i = 1:step_size:length(t)
    h = plot3([p_e_all(i,1), p_tool_all(i,1)], ...
              [p_e_all(i,2), p_tool_all(i,2)], ...
              [p_e_all(i,3), p_tool_all(i,3)], 'Color', [1 0.5 0], 'LineWidth', 1.5);
    if isempty(h_shaft)
        h_shaft = h; 
    end
end

legend('Executed Path', 'Planned Path (offset +3mm Y)', 'Trocar', 'Targets', 'Needle Shaft');

saveas(h_fig1, 'rcm_error.png');
saveas(h_fig2, 'trajectory_3d.png');
disp('Validation plots generated and saved as rcm_error.png and trajectory_3d.png successfully.');

% Indices for each target time.
idx_0  = find(t >= 0, 1)
idx_5  = find(t >= 5, 1);
idx_10 = find(t >= 10, 1);
idx_15 = find(t >= 15, 1);
idx_20 = find(t >= 20, 1);
idx_25 = find(t >= 25, 1);
idx_30 = find(t >= 30, 1);
if isempty(idx_30), idx_30 = length(t); end

% Errors at the target times (mm)
err_points_rcm = [e_rcm_mag(idx_0); e_rcm_mag(idx_5); e_rcm_mag(idx_10); e_rcm_mag(idx_15); e_rcm_mag(idx_20); e_rcm_mag(idx_25); e_rcm_mag(idx_30)] * 1000;
err_points_tool = [e_tool_mag(idx_0); e_tool_mag(idx_5); e_tool_mag(idx_10); e_tool_mag(idx_15); e_tool_mag(idx_20); e_tool_mag(idx_25); e_tool_mag(idx_30)] * 1000;

fprintf('\n=========================================================\n');
fprintf('            ERROR MARGIN AT EACH TARGET POINT\n');
fprintf('=========================================================\n');
fprintf('%-15s | %-10s | %-18s | %-18s\n', 'Target Point', 'Time (s)', 'RCM Error (mm)', 'Tooltip Error (mm)');
fprintf('---------------------------------------------------------\n');
point_names = {'m0 (Start)', 'm1', 'm2', 'm3', 'm4', 'm5 (Start)', 'm5 (End)'};
times_pts = [0, 5, 10, 15, 20, 25, 30];
for k = 1:7
    fprintf('%-15s | %-10.1f | %-18.6f | %-18.6f\n', point_names{k}, times_pts(k), err_points_rcm(k), err_points_tool(k));
end
fprintf('=========================================================\n\n');

% Average and max errors between points
intervals = {
    'm0 -> m1', idx_0, idx_5, '[0, 5]';
    'm1 -> m2', idx_5, idx_10, '[5, 10]';
    'm2 -> m3', idx_10, idx_15, '[10, 15]';
    'm3 -> m4', idx_15, idx_20, '[15, 20]';
    'm4 -> m5', idx_20, idx_25, '[20, 25]';
    'm5 Hold', idx_25, idx_30, '[25, 30]'
};

fprintf('=========================================================================================\n');
fprintf('                         AVERAGE ERROR IN BETWEEN TARGET POINTS\n');
fprintf('=========================================================================================\n');
fprintf('%-12s | %-12s | %-18s | %-18s | %-18s | %-18s\n', ...
    'Transition', 'Interval (s)', 'Avg RCM Err (mm)', 'Max RCM Err (mm)', 'Avg Tooltip Err(mm)', 'Max Tooltip Err(mm)');
fprintf('-----------------------------------------------------------------------------------------\n');

transition_data = struct();
for k = 1:size(intervals, 1)
    name = intervals{k, 1};
    i_start = intervals{k, 2};
    i_end = intervals{k, 3};
    time_str = intervals{k, 4};
    
    sub_rcm = e_rcm_mag(i_start:i_end) * 1000;
    sub_tool = e_tool_mag(i_start:i_end) * 1000;
    
    avg_rcm = mean(sub_rcm);
    max_rcm = max(sub_rcm);
    avg_tool = mean(sub_tool);
    max_tool = max(sub_tool);
    
    fprintf('%-12s | %-12s | %-18.6f | %-18.6f | %-18.6f | %-18.6f\n', ...
        name, time_str, avg_rcm, max_rcm, avg_tool, max_tool);
        
    transition_data(k).name = name;
    transition_data(k).time_str = time_str;
    transition_data(k).avg_rcm = avg_rcm;
    transition_data(k).max_rcm = max_rcm;
    transition_data(k).avg_tool = avg_tool;
    transition_data(k).max_tool = max_tool;
end
fprintf('=========================================================================================\n\n');

fid = fopen('error_statistics.txt', 'w');
if fid ~= -1
    fprintf(fid, 'Target Point Errors Table:\n\n');
    fprintf(fid, '| Target Point | Time (s) | Trocar RCM Error (mm) | Tooltip Tracking Error (mm) |\n');
    fprintf(fid, '| :--- | :--- | :--- | :--- |\n');
    for k = 1:7
        fprintf(fid, '| %s | %d | %.6f | %.6f |\n', point_names{k}, times_pts(k), err_points_rcm(k), err_points_tool(k));
    end
    
    fprintf(fid, '\n\nTransition Errors Table:\n\n');
    fprintf(fid, '| Transition | Time Interval (s) | Avg RCM Error (mm) | Max RCM Error (mm) | Avg Tooltip Error (mm) | Max Tooltip Error (mm) |\n');
    fprintf(fid, '| :--- | :--- | :--- | :--- | :--- | :--- |\n');
    for k = 1:size(intervals, 1)
        fprintf(fid, '| %s | %s | %.6f | %.6f | %.6f | %.6f |\n', ...
            transition_data(k).name, transition_data(k).time_str, ...
            transition_data(k).avg_rcm, transition_data(k).max_rcm, ...
            transition_data(k).avg_tool, transition_data(k).max_tool);
    end
    fclose(fid);
    disp('Error statistics tables written to error_statistics.txt successfully.');
end
