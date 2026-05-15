% connect_visual.m
% Inserts the Visual subsystem from Kuka_Visual.slx into Kuka_CLIK_Model.slx
% and wires it automatically. No manual dragging needed!

projectPath = fileparts(fileparts(mfilename('fullpath')));
simulinkPath = fullfile(projectPath, 'simulink');
addpath(genpath(projectPath));

modelName  = 'Kuka_CLIK_Model';
visualLib  = 'Kuka_Visual';

disp(['=== Connecting Visual subsystem in ', modelName, ' ===']);

% -----------------------------------------------------------------------
% 1. Load both models
% -----------------------------------------------------------------------
if ~bdIsLoaded(modelName)
    load_system(fullfile(simulinkPath, [modelName '.slx']));
end
if ~bdIsLoaded(visualLib)
    load_system(fullfile(simulinkPath, [visualLib '.slx']));
end

set_param(modelName, 'Lock', 'off');

% -----------------------------------------------------------------------
% 2. Remove old blocks if this script is being re-run
% -----------------------------------------------------------------------
blocksToClean = {'Demux_q', 'Visual'};
for k = 1:numel(blocksToClean)
    try, delete_block([modelName '/' blocksToClean{k}]); catch, end
end
try, delete_line(modelName, 'Integrator q/1', 'Demux_q/1'); catch, end

% -----------------------------------------------------------------------
% 3. Add the Visual subsystem FROM the Kuka_Visual library
% -----------------------------------------------------------------------
visualSrc = [visualLib '/Visual'];  % source block in the library
visualDst = [modelName '/Visual'];  % destination in the CLIK model

add_block(visualSrc, visualDst, ...
    'Position', [700 200 900 420], ...
    'MakeNameUnique', 'on');

% -----------------------------------------------------------------------
% 4. Add a Demux (splits the 7x1 integrator output into 7 scalars)
% -----------------------------------------------------------------------
demuxBlk = [modelName '/Demux_q'];
add_block('simulink/Signal Routing/Demux', demuxBlk, ...
    'Outputs', '7', ...
    'Position', [640 228 645 392]);

% -----------------------------------------------------------------------
% 5. Wire: Integrator q --> Demux --> Visual (q1..q7)
% -----------------------------------------------------------------------
ph_int   = get_param([modelName '/Integrator q'], 'PortHandles');
ph_demux = get_param(demuxBlk,   'PortHandles');
ph_vis   = get_param(visualDst,  'PortHandles');

% Integrator -> Demux
Simulink.connectBlocks(ph_int.Outport(1), ph_demux.Inport(1));

% Demux outputs -> Visual ports
for i = 1:7
    Simulink.connectBlocks(ph_demux.Outport(i), ph_vis.Inport(i));
end

% -----------------------------------------------------------------------
% 6. Save
% -----------------------------------------------------------------------
save_system(modelName, fullfile(simulinkPath, [modelName '.slx']));
close_system(visualLib, 0);

disp('=== Done! ===');
disp('Open Kuka_CLIK_Model.slx and press Run to start the 3D simulation!');
