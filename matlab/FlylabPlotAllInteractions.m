function FlylabPlotAllInteractions(interactions, nPre, nPost, nSubsample)
% For the given interactions, plot each one in a subplot, 20 to a figure window.

    nMaxInteractions = 20;
    
    filenamePrev = '';
    clf; 
    iPlot = 0;
    [m,n] = size(interactions);
    for iInteraction=1:m
        filename = interactions{iInteraction,1};
        iStart   = interactions{iInteraction,2};
        iStop    = interactions{iInteraction,3};

        % Read the file if we need to.
        if ~strcmp(filename,filenamePrev)
            filedata = FlylabReadFile(filename); 
        end
        
        if (FlylabIsValidFiledata(filedata))
            iPlot = iPlot+1;
            subplot(4,5,iPlot); 
            iTrigger = iStart;
            FlylabPlotPosition(filedata, 0, [2 3], iTrigger, nSubsample, iStart-nPre, iStop+nPost); 
            axis on; 
            [pth,fn,c] = fileparts(filename);
            txtTitle = fn;
            txtTitle(strfind(txtTitle,'_'))=' '; % Convert underscores to spaces.
            title (sprintf('%s:%d-%d', txtTitle, iStart, iStop)); 
            drawnow; 

            % If the subplots are full, then open another figure window.
            if mod(iPlot,nMaxInteractions)==0
                iPlot=0;
                figure(gcf+1);
                clf;
            end
        end
        
        filenamePrev = filename;
    end
    