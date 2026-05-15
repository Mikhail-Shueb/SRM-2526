%% generate_kuka_library.m
%  Step 2 - Direct Kinematics Simulink Library Generator
%  -------------------------------------------------------
%  This script does the following:
%    1. Gets the robot D-H table from KukaLBR()
%    2. Computes the full symbolic Direct Kinematics transformation T
%    3. Extracts the Rotation matrix R and Position vector p from T
%    4. Creates a Simulink Library called 'Kuka_Lib' with a block that,
%       given the 7 joint angles, outputs R and p.
%
%  HOW TO RUN:
%    - Open MATLAB and set the Current Folder to the 'project' directory.
%    - Type: run('generate_kuka_library.m')  in the command window.
%    - This may take a few minutes because MATLAB needs to do symbolic math
%      for 7 joints. Be patient!

%% ---- Step 0: Setup ----
% Make sure the organized project folders and toolbox are on the path.
projectPath = fileparts(fileparts(mfilename('fullpath')));
toolboxPath = fullfile(fileparts(projectPath), 'toolbox');
simulinkPath = fullfile(projectPath, 'simulink');
addpath(genpath(projectPath));
addpath(toolboxPath);

if ~exist(simulinkPath, 'dir')
    mkdir(simulinkPath);
end

disp('=== KUKA LBR MED - Direct Kinematics Library Generator ===');
disp('Step 1: Loading D-H parameters from KukaLBR()...');

%% ---- Step 1: Load the robot D-H table ----
Robot = KukaLBR();
disp('D-H table loaded successfully. Robot has:');
disp([num2str(size(Robot,1)) ' joints']);

%% ---- Step 2: Compute Direct Kinematics symbolically ----
% DKin() multiplies the 7 individual D-H transformation matrices together:
%   T = A1 * A2 * A3 * A4 * A5 * A6 * A7
% The result T is a 4x4 symbolic matrix.
% This can take 2-5 minutes for a 7-DOF robot.
disp(' ');
disp('Step 2: Computing symbolic Direct Kinematics (this may take a few minutes)...');
tic  % start a timer so you can see how long it takes
Kuka_T = DKin(Robot);
elapsed = toc;
disp(['Done! Took ' num2str(elapsed,'%.1f') ' seconds.']);

%% ---- Step 3: Extract R and p from T ----
% The 4x4 homogeneous transformation T has the structure:
%   T = [ R  p ]
%       [ 0  1 ]
% where R (3x3) is the rotation matrix and p (3x1) is the position vector.
disp(' ');
disp('Step 3: Extracting Rotation matrix (R) and Position vector (p)...');
Kuka_R = Kuka_T(1:3, 1:3);  % top-left 3x3 block
Kuka_p = Kuka_T(1:3, 4);    % top-right 3x1 column

disp('End-effector position vector p (symbolic):');
disp(Kuka_p);

%% ---- Step 4: Create the Simulink Library ----
disp('Step 4: Creating Simulink Library ''Kuka_Lib''...');

% Create a fresh library. If it already exists, close and delete it first.
if bdIsLoaded('Kuka_Lib')
    close_system('Kuka_Lib', 0);
end
libraryFile = fullfile(simulinkPath, 'Kuka_Lib.slx');
if exist(libraryFile, 'file')
    delete(libraryFile);
end

new_system('Kuka_Lib', 'Library');
open_system('Kuka_Lib');

% Generate an optimised MATLAB Function block inside the library.
% Inputs to the block will be q1, q2, q3, q4, q5, q6, q7 (the joint angles).
% Outputs will be Kuka_R (3x3 rotation) and Kuka_p (3x1 position).
disp('Generating Simulink block ''Direct_Kinematics''...');
matlabFunctionBlock('Kuka_Lib/Direct_Kinematics', Kuka_R, Kuka_p, ...
    'FunctionName', 'direct_kinematics_kuka');

%% ---- Step 5: Save the library ----
save_system('Kuka_Lib', libraryFile);
close_system('Kuka_Lib');

disp(' ');
disp(['=== Library saved as ', libraryFile, ' ===']);
disp('You can now open it with: open_system(''Kuka_Lib'')');
disp(' ');
disp('Next step: run validate_direct_kinematics.m to test the model.');
