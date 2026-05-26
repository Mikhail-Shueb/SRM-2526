% Generation of some experiments to evalute CLIK under different conditions

clear; clc;

scriptPath = fileparts(mfilename('fullpath'));
projectPath = fileparts(scriptPath);
figPath = fullfile(projectPath, 'figures');
if ~exist(figPath, 'dir')
    mkdir(figPath);
end

addpath(genpath(projectPath));
addpath(fullfile(fileparts(projectPath), 'toolbox'));

disp('STEP 5 CLIK EXPERIMENTS');

%% Setup

% Parameters
dt = 0.002;
tEnd = 6.0;
t = 0:dt:tEnd;

qRest = zeros(7, 1);
zero3 = zeros(3, 1);

q0 = [0.05; 0.35; 0.15; 0.75; -0.10; -0.30; 0.05];
q0Different = [-0.60; 0.95; 0.70; 0.85; -0.55; 0.65; -0.35];

% Main target pose, generated from a known joint configuration.
qGoal = [0.25; 0.70; -0.35; 1.05; 0.25; -0.55; 0.15];
TGoal = kuka_direct_kinematics(qGoal);
pTarget = TGoal(1:3, 4);
RTarget = TGoal(1:3, 1:3);

%% Point-to-point tests
% Columns: name, q0, pTarget, RTarget, Kp, Ko, K0, lambda
tests = {
    'baseline_K5',            q0,          pTarget, RTarget, 5.0,  4.0,  0.15, 1e-3;
    'low_gain_K15',           q0,          pTarget, RTarget, 1.5,  1.2,  0.15, 1e-3;
    'high_gain_K12',          q0,          pTarget, RTarget, 12.0, 10.0, 0.15, 1e-3;
    'different_q0',           q0Different, pTarget, RTarget, 5.0,  4.0,  0.15, 1e-3;
    'nullspace_off',          q0,          pTarget, RTarget, 5.0,  4.0,  0.00, 1e-3
};

results = struct([]);

for i = 1:size(tests, 1)
    name = tests{i, 1};
    qInit = tests{i, 2};
    pDesired = tests{i, 3};
    RDesired = tests{i, 4};
    KpValue = tests{i, 5};
    KoValue = tests{i, 6};
    K0Value = tests{i, 7};
    lambda = tests{i, 8};

    [qHist, qdotHist, errHist] = run_clik( ...
        qInit, pDesired, RDesired, KpValue, KoValue, K0Value, lambda, ...
        t, dt, qRest, zero3);

    posErr = vecnorm(errHist(1:3, :)) * 1000;
    oriErr = vecnorm(errHist(4:6, :));

    results(i).name = name;
    results(i).Kp = KpValue;
    results(i).Ko = KoValue;
    results(i).K0 = K0Value;
    results(i).lambda = lambda;
    results(i).qHist = qHist;
    results(i).qdotHist = qdotHist;
    results(i).posErr = posErr;
    results(i).oriErr = oriErr;
    results(i).finalPosErr = posErr(end);
    results(i).finalOriErr = oriErr(end);
    results(i).settlingTime = settling_time(t, posErr, oriErr);
    results(i).maxQdot = max(vecnorm(qdotHist));
    results(i).maxJoint = max(abs(qHist), [], 'all');
    results(i).distanceToRest = norm(qHist(:, end) - qRest);

    fprintf('%-24s final pos = %8.4f mm | final ori = %.5g | ts = %.3f s\n', ...
        name, results(i).finalPosErr, results(i).finalOriErr, ...
        results(i).settlingTime);
end

metrics = table( ...
    string({results.name})', ...
    [results.Kp]', ...
    [results.Ko]', ...
    [results.K0]', ...
    [results.lambda]', ...
    [results.finalPosErr]', ...
    [results.finalOriErr]', ...
    [results.settlingTime]', ...
    [results.maxQdot]', ...
    [results.maxJoint]', ...
    [results.distanceToRest]', ...
    'VariableNames', {'case','Kp','Ko','Kn','lambda', ...
    'final_position_error_mm','final_orientation_error_rad','settling_time_s', ...
    'max_joint_speed_rad_s','max_abs_joint_rad', ...
    'null_distance_final_rad'});

writetable(metrics, fullfile(scriptPath, 'step5_clik_experiment_metrics.csv'));

%% Graph 1: proportional gain comparison
figure('Name', 'Gain comparison', 'Visible', 'off');
gainIds = [2, 1, 3]; % low, baseline, high

subplot(2,1,1); hold on;
for id = gainIds
    plot(t, results(id).posErr, 'LineWidth', 1.4);
end
grid on; ylabel('position error [mm]');
legend({results(gainIds).name}, 'Interpreter', 'none');
title('Effect of proportional gains on CLIK convergence');

subplot(2,1,2); hold on;
for id = gainIds
    plot(t, results(id).oriErr, 'LineWidth', 1.4);
end
grid on; xlabel('time [s]'); ylabel('orientation error [rad]');
exportgraphics(gcf, fullfile(figPath, 'step5_gain_convergence.png'), 'Resolution', 180);
close(gcf);

%% Graph 2: null-space comparison
figure('Name', 'Null-space comparison', 'Visible', 'off');
jointLabels = {'q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'q7'};
subplot(2,1,1);
plot(t, results(1).qHist', 'LineWidth', 1.0);
grid on; ylabel('q [rad]'); title('With null-space stabilization');
legend(jointLabels, 'Location', 'eastoutside');

subplot(2,1,2);
plot(t, results(5).qHist', 'LineWidth', 1.0);
grid on; xlabel('time [s]'); ylabel('q [rad]'); title('Without null-space stabilization');
legend(jointLabels, 'Location', 'eastoutside');
exportgraphics(gcf, fullfile(figPath, 'step5_nullspace_joints.png'), 'Resolution', 180);
close(gcf);

disp(' ');
disp('Metrics and figures updated.');

%% Helper functions
function [qHist, qdotHist, errHist] = run_clik(q, pDesired, RDesired, KpValue, KoValue, K0Value, lambda, t, dt, qRest, zero3)
    Kp = KpValue * eye(3);
    Ko = KoValue * eye(3);
    K0 = K0Value * eye(7);

    qHist = zeros(7, numel(t));
    qdotHist = zeros(7, numel(t));
    errHist = zeros(6, numel(t));

    for k = 1:numel(t)
        [qdot, err] = clik_controller(q, pDesired, RDesired, zero3, zero3, ...
                                      Kp, Ko, qRest, K0, 1, lambda);
        q = q + dt * qdot;

        qHist(:, k) = q;
        qdotHist(:, k) = qdot;
        errHist(:, k) = err;
    end
end

function ts = settling_time(t, posErr, oriErr)
    posTolerance = 1.0;   % mm
    oriTolerance = 0.01;  % rad
    ts = NaN;

    insideTolerance = (posErr < posTolerance) & (oriErr < oriTolerance);
    for k = 1:numel(t)
        if all(insideTolerance(k:end))
            ts = t(k);
            return;
        end
    end
end
