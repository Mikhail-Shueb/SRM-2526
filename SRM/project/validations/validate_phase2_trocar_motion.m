% validate_phase2_trocar_motion.m
% Simple smooth movement through target 1, target 2 and target 3.

projectPath = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectPath));

disp('=== KUKA LBR MED - Phase II Smooth Trocar Motion ===');

L = 0.15;
c_r = [0.45; 0.00; 0.35];

targets = [0.50,  0.04, 0.22;
           0.53, -0.03, 0.20;
           0.48,  0.06, 0.18];

segment_time = 3.0;
dt = 0.02;

motion = build_trocar_motion(c_r, targets, L, segment_time, dt);

idx1 = 1;
idx2 = round(segment_time / dt) + 1;
idx3 = numel(motion.t);

target_error = [norm(motion.p_tip(:, idx1) - targets(1, :).'), ...
                norm(motion.p_tip(:, idx2) - targets(2, :).'), ...
                norm(motion.p_tip(:, idx3) - targets(3, :).')];

tool_length_error = abs(vecnorm(motion.p_tip - motion.p_ee) - L);

fprintf('Samples generated          : %d\n', numel(motion.t));
fprintf('Total movement time [s]    : %.2f\n', motion.t(end));
fprintf('Max target error [m]       : %.3e\n', max(target_error));
fprintf('Max tool length error [m]  : %.3e\n', max(tool_length_error));

if max(target_error) < 1e-12 && max(tool_length_error) < 1e-10
    disp('Smooth trocar motion check -> Correct');
else
    error('Smooth trocar motion check failed.');
end

if usejava('desktop')
    figure('Name', 'Phase II - Smooth Trocar Motion');
    hold on; grid on; axis equal;
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title('Tooltip path and end-effector path');

    plot3(motion.p_tip(1, :), motion.p_tip(2, :), motion.p_tip(3, :), ...
          'r-', 'LineWidth', 1.5);
    plot3(motion.p_ee(1, :), motion.p_ee(2, :), motion.p_ee(3, :), ...
          'b-', 'LineWidth', 1.2);
    plot3(targets(:, 1), targets(:, 2), targets(:, 3), ...
          'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
    plot3(c_r(1), c_r(2), c_r(3), ...
          'ks', 'MarkerFaceColor', 'y', 'MarkerSize', 7);

    legend('tooltip path', 'end-effector path', 'targets', 'trocar', ...
           'Location', 'best');
else
    disp('Motion plot skipped in non-desktop MATLAB session.');
end
