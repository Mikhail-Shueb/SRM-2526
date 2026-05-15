% Jacobian generation

% Adding paths
projectPath = fileparts(fileparts(mfilename('fullpath')));
toolboxPath = fullfile(fileparts(projectPath), 'toolbox');
generatedPath = fullfile(projectPath, 'generated');
simulinkPath = fullfile(projectPath, 'simulink');
addpath(genpath(projectPath));
addpath(toolboxPath);

if ~exist(generatedPath, 'dir')
    mkdir(generatedPath);
end
if ~exist(simulinkPath, 'dir')
    mkdir(simulinkPath);
end

disp('=== KUKA LBR MED - Generating Geometric Jacobian ===');

% DH table of the robot
Robot = KukaLBR();

% Define symbolic joint variables
syms q1 q2 q3 q4 q5 q6 q7 real
q_sym = [q1; q2; q3; q4; q5; q6; q7];

% Substitute symbolic variables into the DH table
for i = 1:7
    Robot(i,2) = q_sym(i);
end

disp('Computing Forward Kinematics symbolically...');


T_all = cell(7,1);
T_current = eye(4);

for i = 1:7
    p_params = Robot(i, :);
    A_i = DHTransf(p_params);
    
    T_current = T_current * A_i;
    T_all{i} = T_current;
end

% End-effector position
p_e = T_all{7}(1:3, 4);

disp('Computing Geometric Jacobian J...');

% building the jacobia 6x7 matrix
J_sym = sym(zeros(6, 7));

z0 = [0; 0; 1];
p0 = [0; 0; 0];

for i = 1:7
    if i == 1
        z_prev = z0;
        p_prev = p0;
    else
        z_prev = T_all{i-1}(1:3, 3);
        p_prev = T_all{i-1}(1:3, 4);
    end
    
    % Linear velocity Jacobian (cross product of z-axis and lever arm)
    J_v = cross(z_prev, p_e - p_prev);
    
    % Angular velocity Jacobian (just the z-axis of rotation)
    J_w = z_prev;
    
    J_sym(:, i) = [J_v; J_w];
end

disp('Simplifying Jacobian matrix (this may take a few moments)...');
J_sym = simplify(J_sym);

disp('Saving to Simulink Library (Kuka_Lib)...');

% Generation of simulink library
libraryFile = fullfile(simulinkPath, 'Kuka_Lib.slx');
if exist(libraryFile, 'file') == 4 
    load_system(libraryFile);
    set_param('Kuka_Lib', 'Lock', 'off');
else
    new_system('Kuka_Lib', 'Library');
    load_system('Kuka_Lib');
end

try
    delete_block('Kuka_Lib/Jacobian');
catch
end

% Function block
matlabFunctionBlock('Kuka_Lib/Jacobian', J_sym, ...
                    'Vars', {q_sym}, ...
                    'FunctionName', 'jacobian_kuka');

matlabFunction(J_sym, 'File', fullfile(generatedPath, 'jacobian_kuka.m'), 'Vars', {q_sym});

matlabFunction(T_all{7}, 'File', fullfile(generatedPath, 'kuka_direct_kinematics.m'), 'Vars', {q_sym});

save_system('Kuka_Lib', libraryFile);
close_system('Kuka_Lib');

disp('=== Done! ===');
disp('You can now drag the "Jacobian" block from Kuka_Lib.slx into your models.');
