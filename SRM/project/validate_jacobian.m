% validate_jacobian.m
% Validates the Geometric Jacobian model using numerical differentiation
% and analyzes the effect of kinematic singularities on the Jacobian rank.

% Add toolbox to path
addpath(fullfile(fileparts(mfilename('fullpath')), '..', 'toolbox'));

disp('=== KUKA LBR MED - Geometric Jacobian Validation ===');

% Ensure the jacobian function exists
if exist('kuka_direct_kinematics', 'file') ~= 2
    error('kuka_direct_kinematics.m not found. Please run generate_jacobian_library.m first!');
end

%% =====================================================================
% PART A: Validation via Numerical Differentiation
% ======================================================================
disp(' ');
disp('--- PART A: Numerical Differentiation Validation ---');
disp('Simulating a trajectory to compare analytical J(q)*dq vs numerical dp/dt...');

% Time vector
dt = 0.001;
t = 0:dt:2;

% Simulate a smooth joint trajectory (sine waves of different frequencies)
% q(t) = A * sin(omega * t)
q_traj = zeros(7, length(t));
dq_traj = zeros(7, length(t));

for i = 1:7
    freq = 0.5 + 0.1 * i;   % Different frequencies
    amp = pi/4;             % Amplitude
    
    q_traj(i, :) = amp * sin(2 * pi * freq * t);
    dq_traj(i, :) = amp * 2 * pi * freq * cos(2 * pi * freq * t);
end

% Compute positions over time using Direct Kinematics
p_traj = zeros(3, length(t));

for k = 1:length(t)
    T = kuka_direct_kinematics(q_traj(:, k));
    p_traj(:, k) = T(1:3, 4);
end

% Compute numerical velocity using central differences
v_num = zeros(3, length(t));
for k = 2:(length(t)-1)
    v_num(:, k) = (p_traj(:, k+1) - p_traj(:, k-1)) / (2 * dt);
end

% Compute analytical velocity using Geometric Jacobian (v = J_v * dq)
v_ana = zeros(3, length(t));
for k = 2:(length(t)-1)
    J = jacobian_kuka(q_traj(:, k));
    J_v = J(1:3, :); % Linear velocity part
    
    v_ana(:, k) = J_v * dq_traj(:, k);
end

% Calculate Maximum Error
error_velocity = max(abs(v_num(:, 2:end-1) - v_ana(:, 2:end-1)), [], 'all');

disp(['Maximum Difference between Numerical and Analytical velocity: ', num2str(error_velocity), ' m/s']);
if error_velocity < 1e-4
    disp('✅ SUCCESS: The Geometric Jacobian matches the numerical derivative of the Direct Kinematics!');
else
    disp('❌ FAILURE: High discrepancy found. The Jacobian might be incorrect.');
end

%% =====================================================================
% PART B: Singularity Analysis
% ======================================================================
disp(' ');
disp('--- PART B: Singularity Analysis ---');
disp('Checking the rank of the 6x7 Jacobian matrix at specific singular poses.');
disp('Note: A 7-DOF robot has Redundancy. It only loses rank (drops to 5) if it');
disp('loses a Task-Space DOF (e.g. cannot move outward). If it only loses its');
disp('internal redundancy, the matrix remains Rank 6!');

% 1. Non-Singular (Random) Pose
q_rand = [0.1; 0.5; -0.3; 1.2; 0.4; -0.8; 0.2];
J_rand = jacobian_kuka(q_rand);
r_rand = rank(J_rand, 1e-6);
disp(' ');
disp('1. Generic Non-Singular Pose (e.g. slightly bent):');
disp(['   Rank = ', num2str(r_rand), '  (Expected: 6)']);
if r_rand == 6, disp('   ✅ Full rank achieved. 6 Task-Space DOFs, 1 Redundancy DOF.'); end

% 2. Elbow Singularity (Arm fully stretched)
q_elbow = [0; pi/4; 0; 0; 0; pi/4; 0]; % q4 = 0
J_elbow = jacobian_kuka(q_elbow);
r_elbow = rank(J_elbow, 1e-6);
disp(' ');
disp('2. Elbow Singularity (Arm fully stretched, q4 = 0):');
disp(['   Rank = ', num2str(r_elbow), '  (Expected: 5)']);
if r_elbow < 6, disp('   ✅ Matrix loses rank! The robot cannot push outward (loss of Task-Space DOF).'); end

% 3. Shoulder Singularity (Elbow directly above base)
q_shoulder = [0; 0; pi/4; pi/2; 0; pi/4; 0]; % q2 = 0
J_shoulder = jacobian_kuka(q_shoulder);
r_shoulder = rank(J_shoulder, 1e-6);
disp(' ');
disp('3. Shoulder Singularity (Arm points straight up, q2 = 0):');
disp(['   Rank = ', num2str(r_shoulder), '  (Expected: 6)']);
if r_shoulder == 6, disp('   ✅ Rank is still 6! Joint 1 and 3 align, losing the internal redundancy, but the end-effector can still move in all 6 spatial directions.'); end

% 4. Wrist Singularity (Wrist joints aligned)
q_wrist = [0; pi/4; 0; pi/2; 0; 0; 0]; % q6 = 0
J_wrist = jacobian_kuka(q_wrist);
r_wrist = rank(J_wrist, 1e-6);
disp(' ');
disp('4. Wrist Singularity (Wrist straight, q6 = 0):');
disp(['   Rank = ', num2str(r_wrist), '  (Expected: 6)']);
if r_wrist == 6, disp('   ✅ Rank is still 6! Joint 5 and 7 align, losing internal redundancy, but no loss of spatial mobility.'); end

disp(' ');
disp('=== Validation complete! ===');
