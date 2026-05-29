% build_surgical_simulink.m
% Generates the Simulink model for Phase II Surgical Trajectory Tracking

clear; clc;

projectPath = fileparts(pwd);
workspacePath = fileparts(projectPath);
addpath(fullfile(projectPath, 'generated'));
addpath(fullfile(workspacePath, 'toolbox'));
addpath(pwd); % Ensure 07_phase_II is on path

modelName = 'Surgical_RCM_Sim';
if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
new_system(modelName);

% 1. Add Clock
add_block('simulink/Sources/Clock', [modelName '/Clock']);
set_param([modelName '/Clock'], 'Position', [50, 100, 80, 130]);

% 2. Add Trajectory Planner
add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/Trajectory Planner']);
set_param([modelName '/Trajectory Planner'], 'Position', [150, 70, 280, 160]);

% 3. Add Constant Trocar Position
add_block('simulink/Sources/Constant', [modelName '/Trocar Position']);
set_param([modelName '/Trocar Position'], 'Value', '[-0.5; 0.0; 0.4]');
set_param([modelName '/Trocar Position'], 'Position', [150, 200, 280, 230]);

% 4. Add RCM CLIK Controller
add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/RCM CLIK']);
set_param([modelName '/RCM CLIK'], 'Position', [350, 70, 480, 260]);

% 5. Add Integrator for q
add_block('simulink/Continuous/Integrator', [modelName '/Integrator']);
set_param([modelName '/Integrator'], 'InitialCondition', '[0; 0.4085; 3.1416; 0.6128; -3.1416; 2.1203; 3.1416]');
set_param([modelName '/Integrator'], 'Position', [550, 150, 580, 180]);

% 6. Add Scope for q_dot and q
add_block('simulink/Sinks/Scope', [modelName '/q_dot Scope']);
set_param([modelName '/q_dot Scope'], 'Position', [550, 70, 580, 100]);

add_block('simulink/Sinks/Scope', [modelName '/q Scope']);
set_param([modelName '/q Scope'], 'Position', [650, 150, 680, 180]);

% 7. Add To Workspace for q
add_block('simulink/Sinks/To Workspace', [modelName '/q_workspace']);
set_param([modelName '/q_workspace'], 'Position', [650, 200, 680, 230]);
set_param([modelName '/q_workspace'], 'VariableName', 'q_data');
set_param([modelName '/q_workspace'], 'SaveFormat', 'Array');

% Configure MATLAB Function Blocks
rt = sfroot;

% Planner
block_planner = rt.find('Path', [modelName '/Trajectory Planner'], '-isa', 'Stateflow.EMChart');
planner_code = fileread('trajectory_planner.m');
block_planner.Script = planner_code;

% Controller
block_clik = rt.find('Path', [modelName '/RCM CLIK'], '-isa', 'Stateflow.EMChart');
clik_code = fileread('rcm_clik_controller.m');
block_clik.Script = clik_code;

% Wire them up!
% Clock -> Planner
add_line(modelName, 'Clock/1', 'Trajectory Planner/1');

% Planner -> CLIK
add_line(modelName, 'Trajectory Planner/1', 'RCM CLIK/2'); % p_d
add_line(modelName, 'Trajectory Planner/2', 'RCM CLIK/3'); % v_d

% Trocar -> CLIK
add_line(modelName, 'Trocar Position/1', 'RCM CLIK/4'); % c_r

% Integrator -> CLIK (feedback)
% We have to branch the output of integrator
add_line(modelName, 'Integrator/1', 'RCM CLIK/1'); % q

% CLIK -> Integrator
add_line(modelName, 'RCM CLIK/1', 'Integrator/1');

% CLIK -> Scope
add_line(modelName, 'RCM CLIK/1', 'q_dot Scope/1');

% Integrator -> Scope
add_line(modelName, 'Integrator/1', 'q Scope/1');

% Integrator -> To Workspace
add_line(modelName, 'Integrator/1', 'q_workspace/1');

% Simulation settings
set_param(modelName, 'StopTime', '20.0'); % 3 segments * 5 seconds + 5s hold

save_system(modelName, fullfile(pwd, [modelName '.slx']));
open_system(modelName);

disp('Surgical RCM Simulink Model Generated!');
