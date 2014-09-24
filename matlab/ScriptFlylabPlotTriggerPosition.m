dirspec = '~/FlylabData/zap*';
filespec='zapresponse_TrpA1_sugar_120s_03s_117s_100mW_*.csv';
filenames = GetFilenames(dirspec, filespec);

figure(11); 
mPlot=3;
nPlot=4;
iPlot=0;
%for iFile=1:min(mPlot*nPlot/2,length(filenames))
for iFile=[1,2,5,7,13,22]
%for iFile=1:12
    filedata=FlylabReadFile(filenames{iFile}); 

    nSub=20; 
    nWindow = 1000;
    [iTriggerStart, iTriggerEnd] = FlylabGetTrigger(filedata);
    iStart = iTriggerStart - nWindow;
    iStop  = iTriggerStart + nWindow;

    subplot(mPlot,nPlot,iPlot+1); 
    FlylabPlotPosition(filedata, 0, 2, iTriggerStart, nSub, iStart, iTrig); 
    title(sprintf('%d PRE',iFile));
    xlabel('mm'); ylabel('mm');
    subplot(mPlot,nPlot,iPlot+2); 
    FlylabPlotPosition(filedata, 0, 2, iTriggerStart, nSub, iTrig, iStop);
    title(sprintf('%d POST',iFile));
    xlabel('mm'); ylabel('mm');
    drawnow;
    iPlot = iPlot+2;
end