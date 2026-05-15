% generate_report_figures.m
% Generates and saves high-quality 3D plots for the specific robot poses
% tested in Step 2, 3, and 4. Perfect for the final report.

projectPath = fileparts(mfilename('fullpath'));
toolboxPath = fullfile(fileparts(projectPath), 'toolbox');
addpath(projectPath); addpath(toolboxPath);

% Create figures directory if it doesn't exist
figDir = fullfile(projectPath, 'figures');
if ~exist(figDir, 'dir')
    mkdir(figDir);
end

disp('=== Generating Report Figures ===');

% Define the poses to capture
poses = {
    'DK_1_Home',          [0; 0; 0; 0; 0; 0; 0]
    'DK_2_Shoulder_Bend', [0; pi/2; 0; 0; 0; 0; 0]
    'DK_3_LShape_Bend',   [0; 0; 0; -pi/2; 0; 0; 0]
    'IK_2_Elbow_Bent',    [0; pi/4; 0; pi/3; 0; pi/6; 0]
    'JAC_Elbow_Singular', [0; pi/4; 0; 0; 0; pi/4; 0]
};

% -----------------------------------------------------------------------
% HELPER: Get positions and rotation matrices
% -----------------------------------------------------------------------
function [pts, rots] = get_frame_transforms(q)
    d_vals     = [0; 0; 0.400; 0; 0.400; 0; 0.126];
    a_vals     = zeros(7,1);
    alpha_vals = [pi/2; -pi/2; -pi/2; pi/2; pi/2; -pi/2; 0];
    
    pts  = zeros(3, 8);
    rots = zeros(3, 3, 8);
    rots(:,:,1) = eye(3);
    T = eye(4);
    for j = 1:7
        theta = q(j);
        A = [cos(theta), -sin(theta)*cos(alpha_vals(j)),  sin(theta)*sin(alpha_vals(j)), a_vals(j)*cos(theta);
             sin(theta),  cos(theta)*cos(alpha_vals(j)), -cos(theta)*sin(alpha_vals(j)), a_vals(j)*sin(theta);
             0,           sin(alpha_vals(j)),             cos(alpha_vals(j)),            d_vals(j);
             0,           0,                              0,                             1];
        T = T * A;
        pts(:, j+1)    = T(1:3, 4);
        rots(:,:, j+1) = T(1:3, 1:3);
    end
end

% -----------------------------------------------------------------------
% PLOTTING LOOP
% -----------------------------------------------------------------------
AXIS_LEN = 0.20;
link_colors = [
    0.9  0.9  0.9; 0.2  0.5  1.0; 0.2  0.6  1.0; 0.1  0.7  0.9;
    0.1  0.8  0.8; 0.1  0.9  0.7; 0.2  1.0  0.6; 1.0  0.8  0.2;
];
AXIS_FRAMES = [1, 4, 6, 8];

% Create a single hidden figure for rendering
fig = figure('Name','Report Image Generator','Color','w', 'Position',[100 100 800 800], 'Visible', 'off');
ax = axes('Parent', fig, 'Color','w');

for i = 1:size(poses, 1)
    name = poses{i, 1};
    q    = poses{i, 2};
    
    disp(['Rendering: ', name, ' ...']);
    
    cla(ax); hold(ax,'on'); grid(ax,'on'); axis(ax,'equal');
    view(ax, 45, 25);
    xlabel(ax,'X (m)'); ylabel(ax,'Y (m)'); zlabel(ax,'Z (m)');
    title(ax, strrep(name, '_', ' '), 'Interpreter', 'none', 'FontSize', 14);
    xlim(ax,[-1 1]); ylim(ax,[-1 1]); zlim(ax,[-0.1 1.1]);
    
    % Draw base plate
    th = linspace(0,2*pi,40);
    fill3(ax, 0.08*cos(th), 0.08*sin(th), zeros(1,40), [0.7 0.7 0.7], 'EdgeColor','none');
    
    [pts, rots] = get_frame_transforms(q);
    
    % Draw links and joints
    for j = 1:7
        plot3(ax, pts(1,[j,j+1]), pts(2,[j,j+1]), pts(3,[j,j+1]), '-', 'Color', link_colors(j,:), 'LineWidth', 6);
    end
    for j = 1:8
        plot3(ax, pts(1,j), pts(2,j), pts(3,j), 'o', 'MarkerSize', 10, 'MarkerFaceColor', link_colors(j,:), 'MarkerEdgeColor','k', 'LineWidth',1);
    end
    
    % Draw axes at physical nodes
    for fi = 1:numel(AXIS_FRAMES)
        j = AXIS_FRAMES(fi);
        p = pts(:,j); Rj = rots(:,:,j);
        quiver3(ax, p(1),p(2),p(3), Rj(1,1),Rj(2,1),Rj(3,1), AXIS_LEN, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale','off');
        quiver3(ax, p(1),p(2),p(3), Rj(1,2),Rj(2,2),Rj(3,2), AXIS_LEN, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale','off');
        quiver3(ax, p(1),p(2),p(3), Rj(1,3),Rj(2,3),Rj(3,3), AXIS_LEN, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale','off');
    end
    
    drawnow;
    
    % Save as high-res PNG
    outFile = fullfile(figDir, [name '.png']);
    exportgraphics(ax, outFile, 'Resolution', 300);
end

close(fig);
disp(' ');
disp(['=== Done! Images saved to: ', figDir, ' ===']);
