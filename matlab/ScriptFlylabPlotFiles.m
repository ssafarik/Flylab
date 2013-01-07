% This script plots a set of Flylab files, pausing for a keystroke between each file.
% Main plot is the trajectory.
% Subplots are fly frame trajectory, and heatmap.

dirspec = 'F:\\2013_01_04*';
filespec = '*.csv';

filenames = GetFilenames(dirspec, filespec);
nSubsample = 20;
nPretrigger = -1;
iArena = 0;
iRobot = 1;

for i=1:length(filenames) %[2 16 99 111 116] % 
    [filedata,iTrigger] = FlylabReadData(filenames{i}, nPretrigger); 
    fileheader = FlylabReadHeader(filenames{i});
    iRobots = 1:0+fileheader.robots.nRobots;
    iFlies  = 2:1+fileheader.flies.nFlies;
    
    figure(1); 
    clf; 
    
    subplot(3,2,[1 2 3 4]); 
    FlylabPlotPosition(filedata, iArena, [iRobots,iFlies], iTrigger, nSubsample); 
    title(sprintf('Arena-centered View\nFile %d: %s', i, filenames{i})); 
    %title(sprintf('Arena Centric View')); 
    
    subplot(3,2,5); 
    if fileheader.flies.nFlies>0
        FlylabPlotPosition(filedata, iFlies(1), [iRobots,iFlies], iTrigger, nSubsample); 
    else
        cla;
    end
    title('Fly-centered View');

    %subplot(3,2,6); 
    %FlylabPlotPositionHistogramFiles(dirspec, filespec);
    %title('Heatmap of 5 Trials');

    %drawnow; 
    fprintf ('Press a key for next file %d...\n', i+1);
    pause; 
    fprintf ('working...\n');
end
fprintf ('Done.\n');
