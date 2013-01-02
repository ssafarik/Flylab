% This script plots a set of Flylab files, pausing for a keystroke between each file.
% Main plot is the trajectory.
% Subplots are fly frame trajectory, and heatmap.

dirspec = 'C:\\Flyranch\\2012_12_21*';
filespec = 'getchased*.csv';

filenames = GetFilenames(dirspec, filespec);
nSubsample = 6;
nPretrigger = 9999;

for i=1:length(filenames) %[2 16 99 111 116] % 
    [filedata,iTrigger] = FlylabReadData(filenames{i}, nPretrigger); 
    figure(1); 
    clf; 
    
    subplot(3,2,[1 2 3 4]); 
    FlylabPlotPosition(filedata, 'Arena', iTrigger, nSubsample); 
    title(sprintf('Arena Centric View\nFile %d: %s', i, filenames{i})); 
    %title(sprintf('Arena Centric View')); 
    
    subplot(3,2,5); 
    FlylabPlotPosition(filedata, 'Fly', iTrigger, nSubsample); 
    title('Fly Centric View');

    %subplot(3,2,6); 
    %FlylabPlotPositionHistogramFiles(dirspec, filespec);
    %title('Heatmap of 5 Trials');

    %drawnow; 
    fprintf ('Press a key for next file %d...\n', i+1);
    pause; 
end
