% This script generates a Simulink library for the direct kinematics
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
%   T = A1 * A2 * A3 * A4 * A5 * A6 * A7 -> multiply the homogeneous transformation matrices for each joint to get the overall transformation from base to end-effector
disp(' ');
disp('Step 2: Computing symbolic Direct Kinematics (this may take a few minutes)...');
tic  % timer
Kuka_T = DKin(Robot);
elapsed = toc;
disp(['Done! Took ' num2str(elapsed,'%.1f') ' seconds.']);

%% ---- Step 3: Extract R and p from T ----
%   T = [ R  p ]
%       [ 0  1 ]
disp(' ');
disp('Step 3: Extracting Rotation matrix (R) and Position vector (p)...');
Kuka_R = Kuka_T(1:3, 1:3);  % top-left 3x3 block
Kuka_p = Kuka_T(1:3, 4);    % top-right 3x1 column

disp('End-effector position vector p (symbolic):');
disp(Kuka_p);

%% ---- Step 4: Create the Simulink Library ----
disp('Step 4: Creating Simulink Library ''Kuka_Lib''...');

% Creating of the library
if bdIsLoaded('Kuka_Lib')
    close_system('Kuka_Lib', 0);
end
libraryFile = fullfile(simulinkPath, 'Kuka_Lib.slx');
if exist(libraryFile, 'file')
    delete(libraryFile);
end

new_system('Kuka_Lib', 'Library');
open_system('Kuka_Lib');

% Generation of function block inside the library. The input is the 7 the joint angles.
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
