% validate_clik.m
% Step 5 validation script.

projectPath = fileparts(mfilename('fullpath'));
toolboxPath = fullfile(fileparts(projectPath), 'toolbox');
addpath(projectPath);
addpath(toolboxPath);

disp('=== KUKA LBR MED - Closed Loop Inverse Kinematics Validation ===');

if exist('kuka_direct_kinematics', 'file') ~= 2 || exist('jacobian_kuka', 'file') ~= 2
    error('Run generate_jacobian_library.m first to create kuka_direct_kinematics.m and jacobian_kuka.m.');
end

%% Target pose
q_goal_ref = [0.25; 0.70; -0.35; 1.05; 0.25; -0.55; 0.15];
T_goal = kuka_direct_kinematics(q_goal_ref);
R_d = T_goal(1:3, 1:3);
p_d = T_goal(1:3, 4);

q_ik = inverse_kinematics(R_d, p_d, 0);
T_ik = kuka_direct_kinematics(q_ik);

disp('Analytical IK pose check:');
disp(['  position error [mm] = ', num2str(norm(T_ik(1:3,4) - p_d) * 1000)]);
disp(['  orientation error    = ', num2str(norm(T_ik(1:3,1:3) - R_d, 'fro'))]);

%% Closed-loop integration
dt = 0.002;
t_end = 6.0;
t = 0:dt:t_end;

q = [0.05; 0.35; 0.15; 0.75; -0.10; -0.30; 0.05];
q_rest = zeros(7, 1);

Kp = 5.0 * eye(3);
Ko = 4.0 * eye(3);
Kn = 0.15 * eye(7);
lambda = 1e-3;
method = 1; % damped pseudoinverse

p_dot_d = zeros(3, 1);
omega_d = zeros(3, 1);

err_hist = zeros(6, numel(t));
q_hist = zeros(7, numel(t));

for k = 1:numel(t)
    [q_dot, x_err] = clik_controller(q, p_d, R_d, p_dot_d, omega_d, ...
                                     Kp, Ko, q_rest, Kn, method, lambda);
    q = q + dt * q_dot;
    q_hist(:, k) = q;
    err_hist(:, k) = x_err;
end

T_final = kuka_direct_kinematics(q);
p_final = T_final(1:3, 4);
R_final = T_final(1:3, 1:3);

position_error_mm = norm(p_final - p_d) * 1000;
orientation_error = norm(R_final - R_d, 'fro');
final_task_error = norm(err_hist(:, end));

disp(' ');
disp('CLIK final pose check:');
disp(['  position error [mm] = ', num2str(position_error_mm)]);
disp(['  orientation error    = ', num2str(orientation_error)]);
disp(['  task error norm      = ', num2str(final_task_error)]);
disp(['  final q [rad]        = ', mat2str(q', 4)]);

if position_error_mm < 1.0 && orientation_error < 1e-2
    disp('SUCCESS: CLIK converged to the desired pose.');
else
    disp('WARNING: CLIK did not meet the default tolerance. Tune Kp, Ko, Kn, lambda, or t_end.');
end

%% Plots
figure('Name', 'KUKA LBR MED - CLIK Validation');
subplot(2,1,1);
plot(t, vecnorm(err_hist(1:3, :)) * 1000, 'LineWidth', 1.5);
grid on;
xlabel('time [s]');
ylabel('position error [mm]');
title('CLIK position convergence');

subplot(2,1,2);
plot(t, vecnorm(err_hist(4:6, :)), 'LineWidth', 1.5);
grid on;
xlabel('time [s]');
ylabel('orientation error [rad]');
title('CLIK orientation convergence');
