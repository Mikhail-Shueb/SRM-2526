% visualize_robot_3d.m
% Real-time 3D stick-figure visualisation of the KUKA LBR MED.
% Uses only core MATLAB (plot3/line) - no Sim3D toolbox required.
%
% HOW TO RUN:
%   Option A - Animate from CLIK simulation output:
%       run('validate_clik.m')   % populates q_hist in workspace
%       run('visualize_robot_3d.m')
%
%   Option B - Animate a custom trajectory:
%       Set  q_traj  (7xN matrix) and  dt  below, then run.

projectPath = fileparts(fileparts(mfilename('fullpath')));
toolboxPath = fullfile(fileparts(projectPath), 'toolbox');
addpath(genpath(projectPath));
addpath(toolboxPath);

if exist('kuka_direct_kinematics','file') ~= 2
    error('Run generate_jacobian_library.m first.');
end

% -----------------------------------------------------------------------
% SOURCE OF JOINT ANGLES
% -----------------------------------------------------------------------
if evalin('base','exist(''q_hist'',''var'')')
    q_traj = evalin('base','q_hist');   % from validate_clik.m
    dt     = 0.01;
    disp('Using q_hist from validate_clik.m');
elseif evalin('base','exist(''q_out'',''var'')')
    tmp    = evalin('base','q_out');
    q_traj = tmp.signals.values';      % from Simulink To-Workspace block
    dt     = mean(diff(tmp.time));
    disp('Using q_out from Simulink simulation.');
else
    % Default: animate from home to L-shape over 3 s
    disp('No q_hist or q_out found. Animating a demo trajectory...');
    q_start = zeros(7,1);
    q_end   = [0; pi/4; 0; pi/2; 0; pi/4; 0];
    N  = 200;
    dt = 0.015;
    q_traj = q_start + (q_end - q_start) .* linspace(0,1,N);
end

N = size(q_traj, 2);

% -----------------------------------------------------------------------
% HELPER: Get positions AND rotation matrices for all 8 frames
% -----------------------------------------------------------------------
function [pts, rots] = get_frame_transforms(q)
    d_vals     = [0; 0; 0.400; 0; 0.400; 0; 0.126];
    a_vals     = zeros(7,1);
    alpha_vals = [pi/2; -pi/2; -pi/2; pi/2; pi/2; -pi/2; 0];
    offset_vals = zeros(7,1);

    pts  = zeros(3, 8);
    rots = zeros(3, 3, 8);
    rots(:,:,1) = eye(3);   % base frame = identity

    T = eye(4);
    for j = 1:7
        theta = q(j) + offset_vals(j);
        d     = d_vals(j);
        a     = a_vals(j);
        alp   = alpha_vals(j);
        A = [cos(theta), -sin(theta)*cos(alp),  sin(theta)*sin(alp), a*cos(theta);
             sin(theta),  cos(theta)*cos(alp), -cos(theta)*sin(alp), a*sin(theta);
             0,           sin(alp),             cos(alp),            d;
             0,           0,                    0,                   1];
        T = T * A;
        pts(:, j+1)    = T(1:3, 4);
        rots(:,:, j+1) = T(1:3, 1:3);
    end
end

% -----------------------------------------------------------------------
% SETUP FIGURE
% -----------------------------------------------------------------------
fig = figure('Name','KUKA LBR MED - 3D Visualisation', ...
             'Color','k', 'Position',[100 80 800 700]);
ax  = axes('Parent', fig, 'Color','k', ...
           'XColor',[0.4 0.4 0.4], 'YColor',[0.4 0.4 0.4], 'ZColor',[0.4 0.4 0.4]);
hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
view(ax, 45, 25);
xlabel(ax,'X (m)','Color','w'); ylabel(ax,'Y (m)','Color','w'); zlabel(ax,'Z (m)','Color','w');
title(ax,'KUKA LBR MED — CLIK Simulation','Color','w','FontSize',13);
xlim(ax,[-1 1]); ylim(ax,[-1 1]); zlim(ax,[-0.1 1.1]);

AXIS_LEN = 0.25;   % length of each axis arrow in metres (increase to taste)

% Joint colours
link_colors = [
    0.9  0.9  0.9;
    0.2  0.5  1.0;
    0.2  0.6  1.0;
    0.1  0.7  0.9;
    0.1  0.8  0.8;
    0.1  0.9  0.7;
    0.2  1.0  0.6;
    1.0  0.8  0.2;
];

% Draw base plate
th = linspace(0,2*pi,40);
fill3(ax, 0.08*cos(th), 0.08*sin(th), zeros(1,40), [0.3 0.3 0.3], 'EdgeColor','none');

% Initial pose
[pts, rots] = get_frame_transforms(q_traj(:,1));

% Pre-draw links and joints
hLink  = gobjects(7,1);
hJoint = gobjects(8,1);
for j = 1:7
    hLink(j) = plot3(ax, pts(1,[j,j+1]), pts(2,[j,j+1]), pts(3,[j,j+1]), ...
        '-', 'Color', link_colors(j,:), 'LineWidth', 4);
end
for j = 1:8
    hJoint(j) = plot3(ax, pts(1,j), pts(2,j), pts(3,j), ...
        'o', 'MarkerSize', 9, 'MarkerFaceColor', link_colors(j,:), ...
        'MarkerEdgeColor','w', 'LineWidth',1.2);
end

% Pre-draw coordinate frame axes ONLY at physically distinct nodes.
% Frames 0,1,2 share the base origin; 3&4 share the elbow; 5&6 share the wrist.
% We draw one set of axes per unique position: frames 0, 3, 5, 7.
AXIS_FRAMES = [1, 4, 6, 8];  % 1-indexed into pts/rots (frame 0,3,5,7)
N_AXES = numel(AXIS_FRAMES);
hAxisX = gobjects(N_AXES,1);
hAxisY = gobjects(N_AXES,1);
hAxisZ = gobjects(N_AXES,1);
for fi = 1:N_AXES
    j  = AXIS_FRAMES(fi);
    p  = pts(:,j);
    Rj = rots(:,:,j);
    hAxisX(fi) = quiver3(ax, p(1), p(2), p(3), Rj(1,1), Rj(2,1), Rj(3,1), ...
        AXIS_LEN, 'r', 'LineWidth', 2.0, 'MaxHeadSize', 0.5, 'AutoScale','off');
    hAxisY(fi) = quiver3(ax, p(1), p(2), p(3), Rj(1,2), Rj(2,2), Rj(3,2), ...
        AXIS_LEN, 'g', 'LineWidth', 2.0, 'MaxHeadSize', 0.5, 'AutoScale','off');
    hAxisZ(fi) = quiver3(ax, p(1), p(2), p(3), Rj(1,3), Rj(2,3), Rj(3,3), ...
        AXIS_LEN, 'b', 'LineWidth', 2.0, 'MaxHeadSize', 0.5, 'AutoScale','off');
end

% Legend for axes colours
legend(ax, [hAxisX(1), hAxisY(1), hAxisZ(1)], {'X axis','Y axis','Z axis'}, ...
    'TextColor','w', 'Color','none', 'EdgeColor',[0.4 0.4 0.4], 'Location','northeast');

% End-effector trail
hTrail = plot3(ax, pts(1,8), pts(2,8), pts(3,8), ...
    '-', 'Color',[1 0.6 0 0.6], 'LineWidth', 1.5);
trail_x = pts(1,8); trail_y = pts(2,8); trail_z = pts(3,8);
MAX_TRAIL = 300;

hTime = text(ax, 0.02, 0.95, 0, sprintf('t = %.2f s', 0), ...
    'Units','normalized','Color','w','FontSize',10);

% -----------------------------------------------------------------------
% ANIMATION LOOP
% -----------------------------------------------------------------------
% Enable interactive 3D rotation while animating
rotate3d(ax, 'on');

disp('Animating... you can drag to rotate the view. Close the figure to stop.');
tic;
for k = 1:N
    if ~ishandle(fig), break; end

    [pts, rots] = get_frame_transforms(q_traj(:,k));

    % Update links
    for j = 1:7
        set(hLink(j), 'XData', pts(1,[j,j+1]), ...
                       'YData', pts(2,[j,j+1]), ...
                       'ZData', pts(3,[j,j+1]));
    end
    % Update joints
    for j = 1:8
        set(hJoint(j), 'XData', pts(1,j), 'YData', pts(2,j), 'ZData', pts(3,j));
    end

    % Update frame axes (only at the 4 distinct physical nodes)
    for fi = 1:N_AXES
        j  = AXIS_FRAMES(fi);
        p  = pts(:,j);
        Rj = rots(:,:,j);
        set(hAxisX(fi), 'XData',p(1),'YData',p(2),'ZData',p(3), ...
            'UData',Rj(1,1)*AXIS_LEN,'VData',Rj(2,1)*AXIS_LEN,'WData',Rj(3,1)*AXIS_LEN);
        set(hAxisY(fi), 'XData',p(1),'YData',p(2),'ZData',p(3), ...
            'UData',Rj(1,2)*AXIS_LEN,'VData',Rj(2,2)*AXIS_LEN,'WData',Rj(3,2)*AXIS_LEN);
        set(hAxisZ(fi), 'XData',p(1),'YData',p(2),'ZData',p(3), ...
            'UData',Rj(1,3)*AXIS_LEN,'VData',Rj(2,3)*AXIS_LEN,'WData',Rj(3,3)*AXIS_LEN);
    end

    % Update trail
    trail_x(end+1) = pts(1,8);
    trail_y(end+1) = pts(2,8);
    trail_z(end+1) = pts(3,8);
    if numel(trail_x) > MAX_TRAIL
        trail_x = trail_x(end-MAX_TRAIL+1:end);
        trail_y = trail_y(end-MAX_TRAIL+1:end);
        trail_z = trail_z(end-MAX_TRAIL+1:end);
    end
    set(hTrail,'XData',trail_x,'YData',trail_y,'ZData',trail_z);

    set(hTime, 'String', sprintf('t = %.2f s', (k-1)*dt));

    drawnow limitrate;
    pause(max(0, dt - toc));
    tic;
end
disp('Animation complete!');
