function traj = build_trocar_poses(c_r, targets, L)
% End-effector poses that keep the tool through a trocar.
% c_r: trocar position, 3x1 [m] [x, y, z]
% targets: tooltip target positions, Nx3 or 3xN [m] []
% L: rigid tool length [m]
%
% Output: trajectory parameters struct

% Validations of inputs
if nargin < 3 || isempty(L)
    L = 0.15;
end

c_r = c_r(:);
if numel(c_r) ~= 3
    error('Trocar position c_r must have three coordinates.');
end

targets = normalize_targets(targets);
if L <= 0
    error('Tool length L must be positive.');
end

% Compute the poses for each target
n = size(targets, 2);
p_tip = targets;
p_ee = zeros(3, n);
z_tool = zeros(3, n);
R_d = zeros(3, 3, n);

% For each target, compute the tool axis and end-effector position.
for i = 1:n
    v = p_tip(:, i) - c_r;
    d = norm(v);

    if d < 1e-9
        error('Target %d is at the trocar. Tool axis is undefined.', i);
    end

    z_tool(:, i) = v / d;
    p_ee(:, i) = p_tip(:, i) - L * z_tool(:, i);
    R_d(:, :, i) = make_tool_frame(z_tool(:, i));
end

%  Trajec parameters
traj.L = L;
traj.c_r = c_r;
traj.p_tip = p_tip;
traj.p_ee = p_ee;
traj.z_tool = z_tool;
traj.R_d = R_d;
traj.n_targets = n;
end

function targets = normalize_targets(targets)
if isempty(targets)
    error('At least one target is required.');
end

if size(targets, 2) == 3
    targets = targets.';
    return;
end

if size(targets, 1) == 3
    return;
end

error('Targets must be Nx3 or 3xN.');
end

function R = make_tool_frame(z_axis)
% Pick a stable x-axis that is not parallel to the tool direction.
world_up = [0; 0; 1];
if abs(dot(z_axis, world_up)) > 0.95
    world_up = [0; 1; 0];
end

x_axis = cross(world_up, z_axis);
x_axis = x_axis / norm(x_axis);
y_axis = cross(z_axis, x_axis);
y_axis = y_axis / norm(y_axis);

R = [x_axis, y_axis, z_axis];
end
