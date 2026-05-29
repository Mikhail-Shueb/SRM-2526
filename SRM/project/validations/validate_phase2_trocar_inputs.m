% validate_phase2_trocar_inputs.m
% Checks the first Phase II trajectory step: targets, trocar and tool offset.

projectPath = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(projectPath));

disp('=== KUKA LBR MED - Phase II Trocar Pose Validation ===');

%% Inputs used for the validation
L = 0.15;  % tool length [m]
c_r = [0.45; 0.00; 0.35];

targets = [0.50,  0.04, 0.22;
           0.53, -0.03, 0.20;
           0.48,  0.06, 0.18];

traj = build_trocar_poses(c_r, targets, L);

%% Validation checks
n = traj.n_targets;
axis_error = zeros(1, n);
length_error = zeros(1, n);
rotation_error = zeros(1, n);

for i = 1:n
    trocar_to_tip = traj.p_tip(:, i) - traj.c_r;
    expected_axis = trocar_to_tip / norm(trocar_to_tip);

    axis_error(i) = norm(traj.z_tool(:, i) - expected_axis);
    length_error(i) = abs(norm(traj.p_tip(:, i) - traj.p_ee(:, i)) - L);
    rotation_error(i) = norm(traj.R_d(:, :, i).' * traj.R_d(:, :, i) - eye(3), 'fro');

    if dot(traj.R_d(:, 3, i), expected_axis) < 1 - 1e-10
        error('Tool frame z-axis is not aligned with target %d.', i);
    end
end

fprintf('Targets checked          : %d\n', n);
fprintf('Max tool axis error      : %.3e\n', max(axis_error));
fprintf('Max tool length error [m]: %.3e\n', max(length_error));
fprintf('Max rotation error       : %.3e\n', max(rotation_error));

if max(axis_error) < 1e-10 && max(length_error) < 1e-10 && max(rotation_error) < 1e-10
    disp('Phase II trocar pose check -> Correct');
else
    error('Phase II trocar pose check failed.');
end

%% Check the generated the poses with kinematics
if exist('inverse_kinematics', 'file') == 2 && exist('kuka_direct_kinematics', 'file') == 2
    fk_position_error = zeros(1, n);
    fk_rotation_error = zeros(1, n);
    q_solutions = zeros(7, n);

    for i = 1:n
        q_solutions(:, i) = inverse_kinematics(traj.R_d(:, :, i), traj.p_ee(:, i), 0);
        T_check = kuka_direct_kinematics(q_solutions(:, i));

        fk_position_error(i) = norm(T_check(1:3, 4) - traj.p_ee(:, i));
        fk_rotation_error(i) = norm(T_check(1:3, 1:3) - traj.R_d(:, :, i), 'fro');
    end

    fprintf('Max IK/FK position error [m]: %.3e\n', max(fk_position_error));
    fprintf('Max IK/FK rotation error    : %.3e\n', max(fk_rotation_error));

    if max(fk_position_error) < 1e-6 && max(fk_rotation_error) < 1e-6
        disp('IK/FK round-trip check       -> Correct');
    else
        error('IK/FK round-trip check failed.');
    end
else
    disp('IK/FK round-trip check       -> Skipped, generated kinematics not found.');
end

%% Small plot for visual inspection
if usejava('desktop')
    figure('Name', 'Phase II - Trocar target geometry');
    hold on; grid on; axis equal;
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title('Trocar, tooltip targets and end-effector offsets');

    plot3(c_r(1), c_r(2), c_r(3), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 7);
    plot3(traj.p_tip(1, :), traj.p_tip(2, :), traj.p_tip(3, :), ...
          'rx', 'LineWidth', 1.5, 'MarkerSize', 8);
    plot3(traj.p_ee(1, :), traj.p_ee(2, :), traj.p_ee(3, :), ...
          'bo', 'LineWidth', 1.2, 'MarkerSize', 6);

    for i = 1:n
        plot3([c_r(1), traj.p_tip(1, i)], ...
              [c_r(2), traj.p_tip(2, i)], ...
              [c_r(3), traj.p_tip(3, i)], 'k:');
        quiver3(traj.p_ee(1, i), traj.p_ee(2, i), traj.p_ee(3, i), ...
                traj.z_tool(1, i), traj.z_tool(2, i), traj.z_tool(3, i), ...
                L, 'b', 'LineWidth', 1.2);
    end

    legend('trocar', 'tooltip targets', 'end-effector origins', 'tool line', 'tool axis', ...
           'Location', 'best');
else
    disp('Geometry plot skipped in non-desktop MATLAB session.');
end
