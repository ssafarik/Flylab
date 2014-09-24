% This script plots spectrograms of a set of Flylab files, pausing for a keystroke between each file.

%dirspec = '~/FlylabData/2013_10_24*';
%filespec='zapresponse_TrpA1_sugar_*.csv';

dirspec = '~/FlylabData/zap*';
filespec='zapresponse_HCS_30s_03s_117s_260mW*.csv';

filenames = GetFilenames(dirspec, filespec);
nPretrigger = 1000;
nPosttrigger = 0;
iArena = 0;
iRobot = 1;
iFly = 2;

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

        state = FlylabGetObjectState(filedata, iFly);
        t         = filedata.states(:,1);
        trigger   = filedata.states(:,2);
        x         = state(:,1);
        y         = state(:,2);
        angle     = unwrap(state(:,3));
        vx        = state(:,4);
        vy        = state(:,5);
        vangle    = state(:,6);
        wingleft  = state(:,7);
        wingright = state(:,8);

        s1 = hypot(vx,vy);
        s2 = angle; 
        p1 = s1;
        
        figure(1); 
        clf; 

        subplot(3,1,1);
        spectrogram(s1, 64, 'yaxis');
        txtTitle = filenames{i};
        txtTitle(strfind(txtTitle,'_'))=' '; % Convert underscores to spaces.
        title(sprintf('Spectrogram\nFile %d: %s', i, txtTitle)); 

        subplot(3,1,2);
        spectrogram(s2, 64, 'yaxis');
        txtTitle = filenames{i};
        txtTitle(strfind(txtTitle,'_'))=' '; % Convert underscores to spaces.
        title(sprintf('Spectrogram\nFile %d: %s', i, txtTitle)); 

        subplot(3,1,3);
        plot(p1);
        xlim([0,length(p1)]);

        drawnow; 
    end
    
    if i<length(filenames)
        fprintf ('Press a key for next file %d...\n', i+1);
        pause; 
        fprintf ('working...\n');
    end
end
fprintf ('Done.\n');
