% Build the Simulink 3D visual library for the KUKA model

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

% buildDHActors creates one input port per symbolic joint variable.
syms q1 q2 q3 q4 q5 q6 q7 real

% DH table
DH = [ 0      q1      0      pi/2      0;
       0      q2      0     -pi/2      0;
       0.400  q3      0     -pi/2      0;
       0      q4      0      pi/2      0;
       0.400  q5      0      pi/2      0;
       0      q6      0     -pi/2      0;
       0.126  q7      0      0         0 ];

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

frameWrl = fullfile(sim3dPath, 'frame_axes.wrl');
disp('Building 3D actor blocks for each DH frame...');

buildDHActors(DH, modelName, frameWrl);

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
