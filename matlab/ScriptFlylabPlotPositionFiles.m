% This script plots a set of Flylab files, pausing for a keystroke between each file.
% Main plot is the trajectory.
% Subplots are fly frame trajectory, and heatmap.

%dirspec='C:\\Flyranch\Flylabdata*';
dirspec='F:\\2013_01_02*';
filespec='*.csv';

filenames = GetFilenames(dirspec, filespec);
nSubsample = 5;
nPretrigger = 1000;
iArena = 0;
iRobot = 1;

for i=1:length(filenames)
    [filedata,iTrigger] = FlylabReadData(filenames{i}, nPretrigger); 

    fileheader = FlylabReadHeader(filenames{i});
    iFrameRobots = 1:0+fileheader.robots.nRobots;
    iFrameFlies  = 2:1+fileheader.flies.nFlies;
    
    figure(1); 
    clf; 
    
    subplot(3,2,[1 2 3 4]); 
    FlylabPlotPosition(filedata, iArena, [iFrameRobots,iFrameFlies], iTrigger, nSubsample); 
    title(sprintf('Arena-centered View\nFile %d: %s', i, filenames{i})); 
    
    subplot(3,2,5); 
    if fileheader.flies.nFlies>0
        FlylabPlotPosition(filedata, iFrameFlies(1), [iFrameRobots,iFrameFlies], iTrigger, nSubsample); 
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
