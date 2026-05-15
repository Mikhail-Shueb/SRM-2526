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

projectPath = fileparts(mfilename('fullpath'));
toolboxPath = fullfile(fileparts(projectPath), 'toolbox');
addpath(projectPath); addpath(toolboxPath);

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
% HELPER: Get 3D positions of all 8 frames (base + 7 joints)
% -----------------------------------------------------------------------
function pts = get_frame_positions(q)
    d_vals     = [0; 0; 0.400; 0; 0.400; 0; 0.126];
    a_vals     = zeros(7,1);
    alpha_vals = [pi/2; -pi/2; -pi/2; pi/2; pi/2; -pi/2; 0];
    offset_vals = zeros(7,1);

    pts = zeros(3, 8);  % columns: frame 0 to frame 7
    T = eye(4);
    for j = 1:7
        p_j = [d_vals(j), q(j), a_vals(j), alpha_vals(j), offset_vals(j)];
        % Inline DH matrix (numeric, no symbolic overhead)
        theta = p_j(2);
        d     = p_j(1);
        a     = p_j(3);
        alp   = p_j(4);
        A = [cos(theta), -sin(theta)*cos(alp),  sin(theta)*sin(alp), a*cos(theta);
             sin(theta),  cos(theta)*cos(alp), -cos(theta)*sin(alp), a*sin(theta);
             0,           sin(alp),             cos(alp),            d;
             0,           0,                    0,                   1];
        T = T * A;
        pts(:, j+1) = T(1:3, 4);
    end
end

% -----------------------------------------------------------------------
% SETUP FIGURE
% -----------------------------------------------------------------------
fig = figure('Name','KUKA LBR MED - 3D Visualisation', ...
             'Color','k', 'Position',[100 80 760 680]);
ax  = axes('Parent', fig, 'Color','k', ...
           'XColor',[0.4 0.4 0.4], 'YColor',[0.4 0.4 0.4], 'ZColor',[0.4 0.4 0.4]);
hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
view(ax, 45, 25);
xlabel(ax,'X (m)','Color','w'); ylabel(ax,'Y (m)','Color','w'); zlabel(ax,'Z (m)','Color','w');
title(ax,'KUKA LBR MED — CLIK Simulation','Color','w','FontSize',13);
xlim(ax,[-1 1]); ylim(ax,[-1 1]); zlim(ax,[-0.1 1.1]);

% Joint colours (base=white, links=gradient blue->cyan)
link_colors = [
    0.9  0.9  0.9;   % base
    0.2  0.5  1.0;   % link 1
    0.2  0.6  1.0;   % link 2
    0.1  0.7  0.9;   % link 3
    0.1  0.8  0.8;   % link 4
    0.1  0.9  0.7;   % link 5
    0.2  1.0  0.6;   % link 6 (wrist)
    1.0  0.8  0.2;   % end-effector (gold)
];

% Draw base plate
th = linspace(0,2*pi,40);
fill3(ax, 0.08*cos(th), 0.08*sin(th), zeros(1,40), [0.3 0.3 0.3], 'EdgeColor','none');

% Initial pose
pts = get_frame_positions(q_traj(:,1));

% Pre-draw link lines and joint spheres
hLink = gobjects(7,1);
hJoint = gobjects(8,1);
for j = 1:7
    hLink(j) = plot3(ax, pts(1,[j,j+1]), pts(2,[j,j+1]), pts(3,[j,j+1]), ...
        '-', 'Color', link_colors(j,:), 'LineWidth', 4);
end
for j = 1:8
    hJoint(j) = plot3(ax, pts(1,j), pts(2,j), pts(3,j), ...
        'o', 'MarkerSize', 10, 'MarkerFaceColor', link_colors(j,:), ...
        'MarkerEdgeColor', 'w', 'LineWidth', 1.2);
end

% End-effector trail
hTrail = plot3(ax, pts(1,8), pts(2,8), pts(3,8), ...
    '-', 'Color',[1 0.6 0 0.5], 'LineWidth',1.2);
trail_x = pts(1,8); trail_y = pts(2,8); trail_z = pts(3,8);
MAX_TRAIL = 300;

hTime = text(ax, 0.02, 0.95, 0, sprintf('t = %.2f s', 0), ...
    'Units','normalized','Color','w','FontSize',10);

% -----------------------------------------------------------------------
% ANIMATION LOOP
% -----------------------------------------------------------------------
disp('Animating... close the figure window to stop.');
tic;
for k = 1:N
    if ~ishandle(fig), break; end

    pts = get_frame_positions(q_traj(:,k));

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

    % Update trail
    trail_x(end+1) = pts(1,8);
    trail_y(end+1) = pts(2,8);
    trail_z(end+1) = pts(3,8);
    if numel(trail_x) > MAX_TRAIL
        trail_x = trail_x(end-MAX_TRAIL+1:end);
        trail_y = trail_y(end-MAX_TRAIL+1:end);
        trail_z = trail_z(end-MAX_TRAIL+1:end);
    end
    set(hTrail, 'XData', trail_x, 'YData', trail_y, 'ZData', trail_z);

    set(hTime, 'String', sprintf('t = %.2f s', (k-1)*dt));

    drawnow limitrate;
    pause(max(0, dt - toc));
    tic;
end
disp('Animation complete!');
