% Simple MATLAB-only 3D animation for the KUKA LBR MED.
% The visualization disponibilzed in the Fenix had some issues so we created a backup simple visualizer

projectPath = fileparts(fileparts(mfilename('fullpath')));
toolboxPath = fullfile(fileparts(projectPath), 'toolbox');
addpath(genpath(projectPath));
addpath(toolboxPath);

if exist('kuka_direct_kinematics','file') ~= 2
    error('Run generate_jacobian_library.m first.');
end

if evalin('base','exist(''q_hist'',''var'')')
    q_traj = evalin('base','q_hist');
    dt     = 0.01;
    disp('Using q_hist from validate_clik.m');
elseif evalin('base','exist(''q_out'',''var'')')
    tmp    = evalin('base','q_out');
    q_traj = tmp.signals.values';
    dt     = mean(diff(tmp.time));
    disp('Using q_out from Simulink simulation.');
else
    disp('No q_hist or q_out found. Animating a demo trajectory...');
    q_start = zeros(7,1);
    q_end   = [0; pi/4; 0; pi/2; 0; pi/4; 0];
    N  = 200;
    dt = 0.015;
    q_traj = q_start + (q_end - q_start) .* linspace(0,1,N);
end

N = size(q_traj, 2);

function [pts, rots] = get_frame_transforms(q)
    d_vals     = [0; 0; 0.400; 0; 0.400; 0; 0.126];
    a_vals     = zeros(7,1);
    alpha_vals = [pi/2; -pi/2; -pi/2; pi/2; pi/2; -pi/2; 0];
    offset_vals = zeros(7,1);

    pts  = zeros(3, 8);
    rots = zeros(3, 3, 8);
    rots(:,:,1) = eye(3);

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

fig = figure('Name','KUKA LBR MED - 3D Visualisation', ...
             'Color','k', 'Position',[100 80 800 700]);
ax  = axes('Parent', fig, 'Color','k', ...
           'XColor',[0.4 0.4 0.4], 'YColor',[0.4 0.4 0.4], 'ZColor',[0.4 0.4 0.4]);
hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
view(ax, 45, 25);
xlabel(ax,'X (m)','Color','w'); ylabel(ax,'Y (m)','Color','w'); zlabel(ax,'Z (m)','Color','w');
title(ax,'KUKA LBR MED — CLIK Simulation','Color','w','FontSize',13);
xlim(ax,[-1 1]); ylim(ax,[-1 1]); zlim(ax,[-0.1 1.1]);

AXIS_LEN = 0.25;

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

th = linspace(0,2*pi,40);
fill3(ax, 0.08*cos(th), 0.08*sin(th), zeros(1,40), [0.3 0.3 0.3], 'EdgeColor','none');

[pts, rots] = get_frame_transforms(q_traj(:,1));

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

% Only draw axes at distinct physical locations.
AXIS_FRAMES = [1, 4, 6, 8];
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

legend(ax, [hAxisX(1), hAxisY(1), hAxisZ(1)], {'X axis','Y axis','Z axis'}, ...
    'TextColor','w', 'Color','none', 'EdgeColor',[0.4 0.4 0.4], 'Location','northeast');

hTrail = plot3(ax, pts(1,8), pts(2,8), pts(3,8), ...
    '-', 'Color',[1 0.6 0 0.6], 'LineWidth', 1.5);
trail_x = pts(1,8); trail_y = pts(2,8); trail_z = pts(3,8);
MAX_TRAIL = 300;

hTime = text(ax, 0.02, 0.95, 0, sprintf('t = %.2f s', 0), ...
    'Units','normalized','Color','w','FontSize',10);

rotate3d(ax, 'on');

disp('Animating... you can drag to rotate the view. Close the figure to stop.');
tic;
for k = 1:N
    if ~ishandle(fig), break; end

    [pts, rots] = get_frame_transforms(q_traj(:,k));

    for j = 1:7
        set(hLink(j), 'XData', pts(1,[j,j+1]), ...
                       'YData', pts(2,[j,j+1]), ...
                       'ZData', pts(3,[j,j+1]));
    end

    for j = 1:8
        set(hJoint(j), 'XData', pts(1,j), 'YData', pts(2,j), 'ZData', pts(3,j));
    end

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
