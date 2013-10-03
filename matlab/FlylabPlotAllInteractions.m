function FlylabPlotAllInteractions(interactions, nPre, nPost, nSubsample)
% For the given interactions, plot each one in a subplot, 12 to a figure window.

    nMaxInteractions = 12;
    
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
            subplot(3,4,iPlot); 
            iTrigger = iStart;
            FlylabPlotPosition(filedata, 0, [2 3], iTrigger, nSubsample, iStart-nPre, iStop+nPost); 
            axis on; 
            [pth,fn,c] = fileparts(filename);
            txtTitle = fn;
            txtTitle(strfind(txtTitle,'_'))=' '; % Convert underscores to spaces.
            title (sprintf('%s:%d-%d', txtTitle, iStart, iStop)); 
            drawnow; 

            % If the subplots are full, and there are more to plot, then open another figure window.
            if (mod(iPlot,nMaxInteractions)==0) && (iInteraction<m)
                iPlot=0;
                figure(gcf+1);
                clf;
            end
        end
        
        filenamePrev = filename;
    end
    