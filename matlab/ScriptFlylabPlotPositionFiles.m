% This script plots a set of Flylab files, pausing for a keystroke between each file.
% Main plot is the trajectory.
% Subplots are fly frame trajectory, and heatmap.

%dirspec='C:\\Flyranch\Flylabdata*';
dirspec = '~/FlylabData/2013_02_07*';
filespec='*.csv';

filenames = GetFilenames(dirspec, filespec);
nSubsample = 5;
nPretrigger = 1000;
nPosttrigger = 0;
iArena = 0;
iRobot = 1;

for i=1:length(filenames)
    filedata = FlylabReadFile(filenames{i}); 
    
    % Get the start/trigger/stop indices.
    [iTriggerStart, iTriggerEnd] = FlylabGetTrigger(filedata);
    iStart = max(1, iTriggerStart-nPretrigger);
    [m,n] = size(filedata.states);
    iStop = min(iTriggerEnd+nPosttrigger, m);

    iFrameRobots = 1:0+filedata.header.robots.nRobots;
    iFrameFlies  = 2:1+filedata.header.flies.nFlies;
    
    figure(1); 
    clf; 
    
    subplot(3,2,[1 2 3 4]); 
    FlylabPlotPosition(filedata, iArena, [iFrameRobots,iFrameFlies], iTriggerStart, nSubsample, iStart, iStop); 
    title(sprintf('Arena-centered View\nFile %d: %s', i, filenames{i})); 
    
    subplot(3,2,5); 
    if filedata.header.flies.nFlies>0
        FlylabPlotPosition(filedata, iFrameFlies(1), [iFrameRobots,iFrameFlies], iTriggerStart, nSubsample, iStart, iStop); 
    else
        cla;
    end
    title('Fly-centered View');

    %subplot(3,2,6); 
    %FlylabPlotHistogramPositionFiles(dirspec, filespec, iFrameRobots(1), iFrameFlies(1));
    %title('Heatmap of Trials');

    %drawnow; 
    fprintf ('Press a key for next file %d...\n', i+1);
    pause; 
    fprintf ('working...\n');
end
fprintf ('Done.\n');
