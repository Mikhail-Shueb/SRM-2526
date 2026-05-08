% generate_jacobian_library.m
% Computes the Symbolic Geometric Jacobian for the KUKA LBR MED
% and automatically adds it to the Kuka_Lib Simulink library.

% 1. Add toolbox to path
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'toolbox'));

disp('=== KUKA LBR MED - Generating Geometric Jacobian ===');

% 2. Load Robot DH Table
Robot = KukaLBR();

% Define symbolic joint variables
syms q1 q2 q3 q4 q5 q6 q7 real
q_sym = [q1; q2; q3; q4; q5; q6; q7];

% Replace the joint variables in the DH table with the symbolic variables
% (Since KukaLBR() already puts symbolic variables in column 2, we just extract them)
% Wait, KukaLBR actually uses its own symbolic variables.
% Let's redefine them to make sure we have exactly q_sym
for i = 1:7
    Robot(i,2) = q_sym(i);
end

disp('Computing Forward Kinematics symbolically...');

% 3. Calculate transformations T_0^i for each joint
T_all = cell(7,1);
T_current = eye(4);

for i = 1:7
    % Get DH transformation matrix for this link
    p_params = Robot(i, :);
    A_i = DHTransf(p_params);
    
    % Accumulate transformation
    T_current = T_current * A_i;
    T_all{i} = T_current;
end

% End-effector position (from base)
p_e = T_all{7}(1:3, 4);

disp('Computing Geometric Jacobian J...');
% 4. Build the 6x7 Geometric Jacobian
J_sym = sym(zeros(6, 7));

% Base frame Z axis and origin
z0 = [0; 0; 1];
p0 = [0; 0; 0];

for i = 1:7
    if i == 1
        z_prev = z0;
        p_prev = p0;
    else
        % Extract Z-axis and origin of frame (i-1)
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

% 5. Generate Simulink Block
% Ensure the library exists and is open
if exist('Kuka_Lib', 'file') == 4 % 4 means Simulink model
    load_system('Kuka_Lib');
    set_param('Kuka_Lib', 'Lock', 'off'); % Unlock the library so we can edit it
else
    new_system('Kuka_Lib', 'Library');
    load_system('Kuka_Lib');
end

% Try to remove old block if it exists
try
    delete_block('Kuka_Lib/Jacobian');
catch
    % Block didn't exist, which is fine
end

% Generate MATLAB Function Block in Simulink
matlabFunctionBlock('Kuka_Lib/Jacobian', J_sym, ...
                    'Vars', {q_sym}, ...
                    'FunctionName', 'jacobian_kuka');

% Also generate a standalone .m file so validate_jacobian.m can call it
matlabFunction(J_sym, 'File', 'jacobian_kuka.m', 'Vars', {q_sym});

% Generate the Direct Kinematics .m file for the validation script to use!
matlabFunction(T_all{7}, 'File', 'kuka_direct_kinematics.m', 'Vars', {q_sym});

% Save and close
save_system('Kuka_Lib');
close_system('Kuka_Lib');

disp('=== Done! ===');
disp('You can now drag the "Jacobian" block from Kuka_Lib.slx into your models.');
