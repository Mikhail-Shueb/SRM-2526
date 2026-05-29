% find_initial_configuration.m
% Runs a simulated CLIK loop to converge the robot perfectly to m0 
% and the trocar constraint before we start the actual Simulink simulation.

clear; clc;

projectPath = fileparts(pwd);
addpath(fullfile(projectPath, 'generated'));
addpath(fullfile(projectPath, 'toolbox'));

q = [0; pi/3; 0; -pi/3; 0; pi/3; 0]; % Start with arm pointing roughly downwards
c_r = [-0.6; 0.0; 0.4];
m0 = [-0.6; 0.0; 0.3];
L = 0.15;

dt = 0.01;
K_rcm = 20;
K_tool = 20;

disp('Converging initial configuration...');

for iter = 1:5000
    T_e = kuka_direct_kinematics(q);
    p_e = T_e(1:3, 4);
    R_e = T_e(1:3, 1:3);
    z_e = R_e(:, 3);
    
    J_e = jacobian_kuka(q);
    J_v = J_e(1:3, :);
    J_w = J_e(4:6, :);
    
    % Tool Kinematics
    p_tool = p_e + L * z_e;
    skew_Lz = [0, -L*z_e(3), L*z_e(2); L*z_e(3), 0, -L*z_e(1); -L*z_e(2), L*z_e(1), 0];
    J_tool = J_v - skew_Lz * J_w;
    
    % RCM Kinematics
    lambda = (c_r - p_e)' * z_e;
    p_c = p_e + lambda * z_e;
    e_rcm = c_r - p_c;
    
    skew_lamz = [0, -lambda*z_e(3), lambda*z_e(2); lambda*z_e(3), 0, -lambda*z_e(1); -lambda*z_e(2), lambda*z_e(1), 0];
    J_pc = J_v - skew_lamz * J_w;
    Proj_ortho = eye(3) - z_e * z_e';
    J_rcm = Proj_ortho * J_pc;
    
    % Errors
    e_tool = m0 - p_tool;
    
    if norm(e_rcm) < 1e-4 && norm(e_tool) < 1e-4
        disp(['Converged in ' num2str(iter) ' iterations.']);
        break;
    end
    
    % Control Law
    J_rcm_pinv = pinv(J_rcm);
    q_dot_1 = J_rcm_pinv * (K_rcm * e_rcm);
    
    v_tool_1 = J_tool * q_dot_1;
    v_tool_req = K_tool * e_tool - v_tool_1;
    
    N_1 = eye(7) - J_rcm_pinv * J_rcm;
    J_tool_N = J_tool * N_1;
    J_tool_N_pinv = pinv(J_tool_N);
    
    q_dot_2 = J_tool_N_pinv * v_tool_req;
    q_dot = q_dot_1 + q_dot_2;
    
    q = q + q_dot * dt;
end

disp('Perfect Initial Joint Configuration (q_0):');
disp(mat2str(q, 5));
