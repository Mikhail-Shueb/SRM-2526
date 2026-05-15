% Builds the Simulink model used for the CLIK tests.

% Method 1: damped pseudoinverse
% Method 2: Jacobian transpose


projectPath = fileparts(fileparts(mfilename('fullpath')));
simulinkPath = fullfile(projectPath, 'simulink');
addpath(genpath(projectPath));

if ~exist(simulinkPath, 'dir')
    mkdir(simulinkPath);
end

modelName = 'Kuka_CLIK_Model';
modelFile = fullfile(simulinkPath, [modelName '.slx']);
disp('=== KUKA LBR MED - Generating CLIK Simulink Model ===');

if exist('kuka_direct_kinematics', 'file') ~= 2 || exist('jacobian_kuka', 'file') ~= 2
    error('Run generate_jacobian_library.m first.');
end

assignin('base', 'q_initial', [0.05; 0.35; 0.15; 0.75; -0.10; -0.30; 0.05]);
assignin('base', 'q_rest', zeros(7, 1));
assignin('base', 'p_dot_d', zeros(3, 1));
assignin('base', 'omega_d', zeros(3, 1));
assignin('base', 'Kp_clik', 5.0 * eye(3));
assignin('base', 'Ko_clik', 4.0 * eye(3));
assignin('base', 'Kn_clik', 0.15 * eye(7));
assignin('base', 'lambda_clik', 1e-3);
assignin('base', 'clik_method', 1);

% Use a pose generated from FK so the target is reachable.
q_goal_ref = [0.25; 0.70; -0.35; 1.05; 0.25; -0.55; 0.15];
T_goal = kuka_direct_kinematics(q_goal_ref);
assignin('base', 'p_d', T_goal(1:3, 4));
assignin('base', 'R_d', T_goal(1:3, 1:3));

% Keep the saved model self-contained when opened later.
modelInit = sprintf([ ...
    'modelPath = fileparts(get_param(bdroot, ''FileName''));\n' ...
    'projectPath = fileparts(modelPath);\n' ...
    'addpath(genpath(projectPath));\n' ...
    'q_initial = %s;\n' ...
    'q_rest = zeros(7, 1);\n' ...
    'p_d = %s;\n' ...
    'R_d = %s;\n' ...
    'p_dot_d = zeros(3, 1);\n' ...
    'omega_d = zeros(3, 1);\n' ...
    'Kp_clik = 5.0 * eye(3);\n' ...
    'Ko_clik = 4.0 * eye(3);\n' ...
    'Kn_clik = 0.15 * eye(7);\n' ...
    'lambda_clik = 1e-3;\n' ...
    'clik_method = 1;\n'], ...
    mat2str(evalin('base', 'q_initial'), 16), ...
    mat2str(T_goal(1:3, 4), 16), ...
    mat2str(T_goal(1:3, 1:3), 16));

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end
if exist(modelFile, 'file')
    delete(modelFile);
end

new_system(modelName);
open_system(modelName);
set_param(modelName, 'InitFcn', modelInit);

% Inputs and gains.
add_block('simulink/Sources/Constant', [modelName '/p_d'], ...
          'Value', 'p_d', 'Position', [30 40 100 70]);
add_block('simulink/Sources/Constant', [modelName '/R_d'], ...
          'Value', 'R_d', 'Position', [30 95 100 125]);
add_block('simulink/Sources/Constant', [modelName '/p_dot_d'], ...
          'Value', 'p_dot_d', 'Position', [30 150 100 180]);
add_block('simulink/Sources/Constant', [modelName '/omega_d'], ...
          'Value', 'omega_d', 'Position', [30 205 100 235]);
add_block('simulink/Sources/Constant', [modelName '/Kp'], ...
          'Value', 'Kp_clik', 'Position', [30 260 100 290]);
add_block('simulink/Sources/Constant', [modelName '/Ko'], ...
          'Value', 'Ko_clik', 'Position', [30 315 100 345]);
add_block('simulink/Sources/Constant', [modelName '/q_rest'], ...
          'Value', 'q_rest', 'Position', [30 370 100 400]);
add_block('simulink/Sources/Constant', [modelName '/Kn'], ...
          'Value', 'Kn_clik', 'Position', [30 425 100 455]);
add_block('simulink/Sources/Constant', [modelName '/method'], ...
          'Value', 'clik_method', 'Position', [30 480 100 510]);
add_block('simulink/Sources/Constant', [modelName '/lambda'], ...
          'Value', 'lambda_clik', 'Position', [30 535 100 565]);

% Controller loop and outputs.
add_block('simulink/Continuous/Integrator', [modelName '/Integrator q'], ...
          'InitialCondition', 'q_initial', 'Position', [520 250 560 290]);
add_block('simulink/User-Defined Functions/MATLAB Function', [modelName '/CLIK Controller'], ...
          'Position', [230 190 380 410]);
add_block('simulink/Sinks/Scope', [modelName '/q scope'], ...
          'Position', [650 235 700 285]);
add_block('simulink/Sinks/Scope', [modelName '/error scope'], ...
          'Position', [650 335 700 385]);
add_block('simulink/Sinks/To Workspace', [modelName '/q_out'], ...
          'VariableName', 'q_out', 'SaveFormat', 'StructureWithTime', ...
          'Position', [650 285 735 315]);
add_block('simulink/Sinks/To Workspace', [modelName '/x_err_out'], ...
          'VariableName', 'x_err_out', 'SaveFormat', 'StructureWithTime', ...
          'Position', [650 390 755 420]);

rt = sfroot;
chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', [modelName '/CLIK Controller']);
chart.Script = sprintf([ ...
    'function [q_dot, x_err] = fcn(q, p_d, R_d, p_dot_d, omega_d, Kp, Ko, q_rest, Kn, method, lambda)\n' ...
    '%%#codegen\n' ...
    '[q_dot, x_err] = clik_controller(q, p_d, R_d, p_dot_d, omega_d, Kp, Ko, q_rest, Kn, method, lambda);\n' ...
    'end\n']);

% Wire the loop.
add_line(modelName, 'Integrator q/1', 'CLIK Controller/1');
add_line(modelName, 'p_d/1', 'CLIK Controller/2');
add_line(modelName, 'R_d/1', 'CLIK Controller/3');
add_line(modelName, 'p_dot_d/1', 'CLIK Controller/4');
add_line(modelName, 'omega_d/1', 'CLIK Controller/5');
add_line(modelName, 'Kp/1', 'CLIK Controller/6');
add_line(modelName, 'Ko/1', 'CLIK Controller/7');
add_line(modelName, 'q_rest/1', 'CLIK Controller/8');
add_line(modelName, 'Kn/1', 'CLIK Controller/9');
add_line(modelName, 'method/1', 'CLIK Controller/10');
add_line(modelName, 'lambda/1', 'CLIK Controller/11');
add_line(modelName, 'CLIK Controller/1', 'Integrator q/1');
add_line(modelName, 'Integrator q/1', 'q scope/1');
add_line(modelName, 'Integrator q/1', 'q_out/1');
add_line(modelName, 'CLIK Controller/2', 'error scope/1');
add_line(modelName, 'CLIK Controller/2', 'x_err_out/1');

set_param(modelName, 'StopTime', '6');
set_param(modelName, 'Solver', 'ode4', 'FixedStep', '0.002');

save_system(modelName, modelFile);
close_system(modelName);

disp(['Saved ', modelFile]);
disp('Run sim(''Kuka_CLIK_Model'') to validate the closed-loop behavior.');
