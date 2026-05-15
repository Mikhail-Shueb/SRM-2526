% generate_kuka_visual.m
% Generates a Simulink 3D visualisation of the KUKA LBR MED using the
% RobotX_sim3d package provided by the professor.
%
% HOW TO RUN:
%   1. Run generate_jacobian_library.m first (if you haven't already).
%   2. Run this script.
%   3. Open Kuka_CLIK_Model.slx and Kuka_Visual.slx side by side.
%   4. Run the simulation - the 3D viewer will update in real time!

% -----------------------------------------------------------------------
% 1. Setup paths
% -----------------------------------------------------------------------
projectPath  = fileparts(fileparts(mfilename('fullpath')));
sim3dPath    = fullfile(fileparts(projectPath), 'RobotX_sim3d');
toolboxPath  = fullfile(fileparts(projectPath), 'toolbox');
simulinkPath = fullfile(projectPath, 'simulink');

addpath(genpath(projectPath));
addpath(sim3dPath);
addpath(toolboxPath);

if ~exist(simulinkPath, 'dir')
    mkdir(simulinkPath);
end

disp('=== KUKA LBR MED - Generating 3D Visualisation ===');

% -----------------------------------------------------------------------
% 2. Build a *numeric template* DH table for buildDHActors.
%    buildDHActors needs a table where each joint's symbolic variable
%    is replaced by a fresh syms variable q1..q7 (one per row).
%    The function will create an Input port in the subsystem for each one.
% -----------------------------------------------------------------------
syms q1 q2 q3 q4 q5 q6 q7 real

% KUKA LBR MED DH table  [d      theta   a      alpha     offset]
DH = [ 0      q1      0      pi/2      0;
       0      q2      0     -pi/2      0;
       0.400  q3      0     -pi/2      0;
       0      q4      0      pi/2      0;
       0.400  q5      0      pi/2      0;
       0      q6      0     -pi/2      0;
       0.126  q7      0      0         0 ];

% -----------------------------------------------------------------------
% 3. Create (or recreate) the Kuka_Visual library
% -----------------------------------------------------------------------
modelName = 'Kuka_Visual';

if bdIsLoaded(modelName)
    close_system(modelName, 0);
end

slxFile = fullfile(simulinkPath, [modelName '.slx']);
if exist(slxFile, 'file')
    delete(slxFile);
end

new_system(modelName, 'Library');
open_system(modelName);

% -----------------------------------------------------------------------
% 4. Call buildDHActors - this creates the 'Visual' subsystem with
%    7 input ports (q1..q7) and one 3D actor per DH frame.
%    It requires the frame_axes.wrl file which is in the sim3d folder.
% -----------------------------------------------------------------------
frameWrl = fullfile(sim3dPath, 'frame_axes.wrl');
disp('Building 3D actor blocks for each DH frame...');

buildDHActors(DH, modelName, frameWrl);

% -----------------------------------------------------------------------
% 5. Save the library
% -----------------------------------------------------------------------
save_system(modelName, slxFile);
close_system(modelName);

disp(['=== Done! Saved as ', slxFile, ' ===']);
disp(' ');
disp('NEXT STEPS:');
disp('  1. Open Kuka_CLIK_Model.slx');
disp('  2. Open Kuka_Visual.slx');
disp('  3. Drag the "Visual" subsystem from Kuka_Visual into Kuka_CLIK_Model.');
disp('  4. Connect the "Integrator q" output (the 7x1 joint vector) to the');
disp('     Demux block -> then wire each q_i to the matching input port');
disp('     of the Visual subsystem (q1 -> 1, q2 -> 2, ... q7 -> 7).');
disp('  5. Press Run - the Sim3D viewer will open automatically!');
