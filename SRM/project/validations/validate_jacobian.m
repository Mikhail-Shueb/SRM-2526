% validate_jacobian.m
% Jacobian checks: velocity match and rank at selected poses.

projectPath = fileparts(fileparts(mfilename('fullpath')));
toolboxPath = fullfile(fileparts(projectPath), 'toolbox');
addpath(genpath(projectPath));
addpath(toolboxPath);

disp('=== KUKA LBR MED - Geometric Jacobian Validation ===');

if exist('kuka_direct_kinematics', 'file') ~= 2
    error('kuka_direct_kinematics.m not found. Please run generate_jacobian_library.m first!');
end

%% Velocity check
disp(' ');
disp('--- PART A: Velocity Validation (Analytical vs Numerical) ---');

dt = 0.001;
t = 0:dt:2;

% Smooth test trajectory.
q_traj = zeros(7, length(t));
dq_traj = zeros(7, length(t));

for i = 1:7
    freq = 0.5 + 0.1 * i;
    amp = pi/4;
    
    q_traj(i, :) = amp * sin(2 * pi * freq * t);
    dq_traj(i, :) = amp * 2 * pi * freq * cos(2 * pi * freq * t);
end

p_traj = zeros(3, length(t));
R_traj = zeros(3, 3, length(t));

for k = 1:length(t)
    T = kuka_direct_kinematics(q_traj(:, k));
    p_traj(:, k) = T(1:3, 4);
    R_traj(:, :, k) = T(1:3, 1:3);
end

v_num = zeros(3, length(t));
omega_num = zeros(3, length(t));

for k = 2:(length(t)-1)
    v_num(:, k) = (p_traj(:, k+1) - p_traj(:, k-1)) / (2 * dt);
    
    dR_dt = (R_traj(:, :, k+1) - R_traj(:, :, k-1)) / (2 * dt);
    S_omega = dR_dt * R_traj(:, :, k)';
    
    omega_num(1, k) = S_omega(3, 2);
    omega_num(2, k) = S_omega(1, 3);
    omega_num(3, k) = S_omega(2, 1);
end

v_ana = zeros(3, length(t));
omega_ana = zeros(3, length(t));

for k = 2:(length(t)-1)
    J = jacobian_kuka(q_traj(:, k));
    J_v = J(1:3, :);
    J_w = J(4:6, :);
    
    v_ana(:, k) = J_v * dq_traj(:, k);
    omega_ana(:, k) = J_w * dq_traj(:, k);
end

error_linear = max(abs(v_num(:, 2:end-1) - v_ana(:, 2:end-1)), [], 'all');
error_angular = max(abs(omega_num(:, 2:end-1) - omega_ana(:, 2:end-1)), [], 'all');

if error_linear < 1e-4 && error_angular < 5e-3
    disp('Linear/Angular Velocity Match -> Correct');
else
    disp('Linear/Angular Velocity Match -> Wrong');
end

%% Rank checks
disp(' ');
disp('--- PART B: Singularity Analysis (Jacobian Rank) ---');

% Generic pose
q_rand = [0.1; 0.5; -0.3; 1.2; 0.4; -0.8; 0.2];
J_rand = jacobian_kuka(q_rand);
r_rand = rank(J_rand, 1e-6);
if r_rand == 6; res = 'Correct'; else; res = 'Wrong'; end
disp(['1. Generic Pose: Rank ', num2str(r_rand), ' -> ', res]);

% Elbow singularity
q_elbow = [0; pi/4; 0; 0; 0; pi/4; 0];
J_elbow = jacobian_kuka(q_elbow);
r_elbow = rank(J_elbow, 1e-6);
if r_elbow == 5; res = 'Correct'; else; res = 'Wrong'; end
disp(['2. Elbow Singularity: Rank ', num2str(r_elbow), ' -> ', res]);

% Shoulder singularity
q_shoulder = [0; 0; pi/4; pi/2; 0; pi/4; 0];
J_shoulder = jacobian_kuka(q_shoulder);
r_shoulder = rank(J_shoulder, 1e-6);
if r_shoulder == 6; res = 'Correct'; else; res = 'Wrong'; end
disp(['3. Shoulder Singularity: Rank ', num2str(r_shoulder), ' -> ', res]);

% Wrist singularity
q_wrist = [0; pi/4; 0; pi/2; 0; 0; 0];
J_wrist = jacobian_kuka(q_wrist);
r_wrist = rank(J_wrist, 1e-6);
if r_wrist == 6; res = 'Correct'; else; res = 'Wrong'; end
disp(['4. Wrist Singularity: Rank ', num2str(r_wrist), ' -> ', res]);

disp(' ');
disp('=== Validation complete! ===');
