% solve_arc_trajectory.m
% Simulates the arc trajectory and generates a Simulink model for it.

clear; clc; close all;

%% 1. Mathematical Deduction of the Trajectory
% Time parameters
tf = 5;
t = linspace(0, tf, 100);
R = 1; % Arbitrary radius for simulation

% Cubic timing law: s(t) = a0 + a1*t + a2*t^2 + a3*t^3
% Conditions: s(0)=0, s_dot(0)=0, s(tf)=1, s_dot(tf)=0
% Result: s(t) = 3*(t/tf)^2 - 2*(t/tf)^3
s = 3*(t/tf).^2 - 2*(t/tf).^3;

% The angle phi goes from 0 to pi/2
phi = (pi/2) * s;

% Position Trajectory
% The arc is in the Y-Z plane. 
% Starts at (0, 0, R) and ends at (0, R, 0)
p = zeros(3, length(t));
p(1, :) = 0;
p(2, :) = R * sin(phi);
p(3, :) = R * cos(phi);

% Orientation Trajectory
% y_e is tangent to the arc: [0; cos(phi); -sin(phi)]
% z_e is normal to the arc:  [0; sin(phi); cos(phi)]
% x_e = y_e x z_e = [1; 0; 0]
% This corresponds to a rotation around the X-axis by angle -phi.
% Vector-angle representation:
k = [-1; 0; 0]; % Fixed rotation axis
theta = phi;    % Angle of rotation over time

%% 2. Visualization of the Trajectory
figure('Name', 'Arc Trajectory Simulation', 'Color', 'w');
hold on; grid on; axis equal;
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k', 'GridColor', 'k', 'GridAlpha', 0.3);
view(3);
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Arc Trajectory with Tangent Frame');

% Plot the path
plot3(p(1,:), p(2,:), p(3,:), 'k-', 'LineWidth', 2);

% Plot coordinate frames at specific intervals
step = 20;
for i = 1:step:length(t)
    % Rotation matrix at time i
    c = cos(-phi(i)); s_ang = sin(-phi(i));
    R_mat = [1, 0, 0; 
             0, c, -s_ang; 
             0, s_ang, c];
    
    pos = p(:, i);
    
    % X axis (Red)
    quiver3(pos(1), pos(2), pos(3), R_mat(1,1), R_mat(2,1), R_mat(3,1), 0.2, 'r', 'LineWidth', 2);
    % Y axis (Green) - Tangent
    quiver3(pos(1), pos(2), pos(3), R_mat(1,2), R_mat(2,2), R_mat(3,2), 0.2, 'g', 'LineWidth', 2);
    % Z axis (Blue) - Normal
    quiver3(pos(1), pos(2), pos(3), R_mat(1,3), R_mat(2,3), R_mat(3,3), 0.2, 'b', 'LineWidth', 2);
end
view(90, 0); % Look directly at the Y-Z plane

%% 3. Generate Simulink Model
modelName = 'Arc_Trajectory_Sim';
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
new_system(modelName);

% Add blocks
add_block('simulink/Sources/Clock', [modelName '/Clock']);
add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/Trajectory']);
add_block('simulink/Sinks/Scope', [modelName '/Position Scope']);
add_block('simulink/Sinks/Scope', [modelName '/Angle Scope']);

% Position the blocks
set_param([modelName '/Clock'], 'Position', [50, 100, 80, 130]);
set_param([modelName '/Trajectory'], 'Position', [150, 80, 250, 150]);
set_param([modelName '/Position Scope'], 'Position', [320, 70, 350, 100]);
set_param([modelName '/Angle Scope'], 'Position', [320, 130, 350, 160]);
% Set MATLAB Function code
% We use sfroot to access the Stateflow chart behind the MATLAB Function block
rt = sfroot;
block = rt.find('Path', [modelName '/Trajectory'], '-isa', 'Stateflow.EMChart');

scriptCode = sprintf([ ...
    'function [p, k, theta] = fcn(t)\n', ...
    '    %% Trajectory parameters\n', ...
    '    tf = 5;\n', ...
    '    R = 1;\n\n', ...
    '    %% Cubic timing law\n', ...
    '    if t <= 0\n', ...
    '        s = 0;\n', ...
    '    elseif t >= tf\n', ...
    '        s = 1;\n', ...
    '    else\n', ...
    '        s = 3*(t/tf)^2 - 2*(t/tf)^3;\n', ...
    '    end\n\n', ...
    '    %% Angle along the arc\n', ...
    '    phi = (pi/2) * s;\n\n', ...
    '    %% Position [x; y; z]\n', ...
    '    p = [0; R*sin(phi); R*cos(phi)];\n\n', ...
    '    %% Orientation (Vector-Angle)\n', ...
    '    k = [-1; 0; 0];\n', ...
    '    theta = phi;\n', ...
    'end' ...
]);

block.Script = scriptCode;

% Connect blocks AFTER the script is set so the ports are created
add_line(modelName, 'Clock/1', 'Trajectory/1');
add_line(modelName, 'Trajectory/1', 'Position Scope/1');
add_line(modelName, 'Trajectory/3', 'Angle Scope/1');

% Configure simulation time
set_param(modelName, 'StopTime', '5.0');

% Save and open
save_system(modelName, fullfile(pwd, [modelName '.slx']));
open_system(modelName);

disp('Trajectory simulated and Simulink model generated successfully!');
