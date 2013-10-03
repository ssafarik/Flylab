% This script plots a set of Flylab files, pausing for a keystroke between each file.
% Main plot is the trajectory.
% Subplots are fly frame trajectory, and heatmap.

%dirspec='C:\\Flyranch\Flylabdata*';
dirspec = '~/FlylabData/2013_09_25*';
filespec='vel_UAS_TrpA1_geosmin_120mW_*.csv';

filenames = GetFilenames(dirspec, filespec);
nSubsample = 5;
nPretrigger = 1000;
nPosttrigger = 0;
iArena = 0;
iRobot = 1;

for i=1:length(filenames)
    filedata = FlylabReadFile(filenames{i}); 
    if (size(fieldnames(filedata.header))>0)
        % Get the start/trigger/stop indices.
        [iTriggerStart, iTriggerEnd] = FlylabGetTrigger(filedata);
        iStart = max(1, iTriggerStart-nPretrigger);
        [m,n] = size(filedata.states);
        iStop = min(iTriggerEnd+nPosttrigger, m);

        iFrameRobots = 1:0+filedata.header.robotspec.nRobots;
        iFrameFlies  = 2:1+filedata.header.flyspec.nFlies;

        figure(1); 
        clf; 

        subplot(3,2,[1 2 3 4]); 
        FlylabPlotPosition(filedata, iArena, [iFrameRobots,iFrameFlies], iTriggerStart, nSubsample, iStart, iStop); 
        txtTitle = filenames{i};
        txtTitle(strfind(txtTitle,'_'))=' '; % Convert underscores to spaces.
        title(sprintf('Arena-centered View\nFile %d: %s', i, txtTitle)); 

        subplot(3,2,5); 
        if filedata.header.flyspec.nFlies>0
            FlylabPlotPosition(filedata, iFrameFlies(1), [iFrameRobots,iFrameFlies], iTriggerStart, nSubsample, iStart, iStop); 
        else
            cla;
        end
        title('Fly-centered View');

        %subplot(3,2,6); 
        %FlylabPlotHistogramPositionFiles(dirspec, filespec, iFrameRobots(1), iFrameFlies(1));
        %title('Heatmap of Trials');

        %drawnow; 
    end
    
    if i<length(filenames)
        fprintf ('Press a key for next file %d...\n', i+1);
        pause; 
        fprintf ('working...\n');
    end
end
fprintf ('Done.\n');
