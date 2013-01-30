function FlylabPlotAllInteractions(interactions, nPre, nPost, nSubsample)
% For the given interactions, plot each one in a subplot, 20 to a figure window.

    nMaxInteractions = 20;
    
    filenamePrev = '';
    clf; 
    iPlot = 0;
    for iInteraction=1:length(interactions)
        filename = interactions{iInteraction,1};
        iStart   = interactions{iInteraction,2};
        iStop    = interactions{iInteraction,3};

        % Read the file if we need to.
        if ~strcmp(filename,filenamePrev)
            [filedata,iTrigger] = FlylabReadData(filename, -1); 
        end
        
        iPlot = iPlot+1;
        subplot(4,5,iPlot); 
        FlylabPlotPosition(filedata(iStart:iStop,:), 0, [2 3], nPre, nSubsample); 
        axis on; 
        [pth,fn,c] = fileparts(filename);
        title (sprintf('%s:%d-%d', fn, iStart, iStop)); 
        drawnow; 

        % If the subplots are full, then open another figure window.
        if mod(iPlot,nMaxInteractions)==0
            iPlot=0;
            figure(gcf+1);
            clf;
        end
        
        filenamePrev = filename;
    end
    