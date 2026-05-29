function motion = build_trocar_motion(c_r, targets, L, segment_time, dt)
%BUILD_TROCAR_MOTION Smooth tooltip motion through all targets.
% 
% c_r: trocar position, 3x1 [m] [x, y, z]
% targets: tooltip target positions, Nx3 or 3xN [m] []
% L: rigid tool length [m]
% segment_time: time to move between each pair of targets [s]
% dt: time step for sampling the motion [s]
% 
% Output: motion struct with fields:

% Validations
if nargin < 4 || isempty(segment_time)
    segment_time = 3.0;
end
if nargin < 5 || isempty(dt)
    dt = 0.02;
end

targets = normalize_targets(targets);
n_targets = size(targets, 2);
if n_targets < 2
    error('At least two targets are needed for a movement.');
end

t_all = [];
p_tip_all = [];

% Generate smooth trajectories between consecutive targets using a cube polynomial
for i = 1:(n_targets - 1)
    t_local = 0:dt:segment_time;
    if i > 1
        t_local = t_local(2:end);
    end

    tau = t_local / segment_time;
    s = 3*tau.^2 - 2*tau.^3; % Cubic polynomial for smooth start and stop

    p0 = targets(:, i);
    p1 = targets(:, i + 1);
    p_tip = p0 + (p1 - p0) .* s;

    t_start = (i - 1) * segment_time;
    t_all = [t_all, t_start + t_local];
    p_tip_all = [p_tip_all, p_tip]; 
end

traj = build_trocar_poses(c_r, p_tip_all, L);

motion.t = t_all;
motion.p_tip = traj.p_tip;
motion.p_ee = traj.p_ee;
motion.R_d = traj.R_d;
motion.z_tool = traj.z_tool;
motion.c_r = traj.c_r;
motion.L = traj.L;
motion.targets = targets;
end

function targets = normalize_targets(targets)
if size(targets, 2) == 3
    targets = targets.';
    return;
end
if size(targets, 1) == 3
    return;
end
error('Targets must be Nx3 or 3xN.');
end
