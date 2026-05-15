% startup.m
% Adds the organized KUKA LBR MED project folders to the MATLAB path.

projectPath = fileparts(mfilename('fullpath'));
toolboxPath = fullfile(fileparts(projectPath), 'toolbox');

addpath(genpath(projectPath));
addpath(toolboxPath);

disp('KUKA LBR MED project paths loaded.');
