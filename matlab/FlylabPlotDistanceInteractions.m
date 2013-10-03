function FlylabPlotDistanceInteractions(interactions, nPre, nPost)
% For the given interactions, plot the average distance over all the interactions.

    clf;
    
    % Create an array to hold the longest interaction.
    nMinLen = intmax;
    nMaxLen = 0;
    [m n] = size(interactions);
    for iInteraction=1:m
        iStart   = interactions{iInteraction,2};
        iStop    = interactions{iInteraction,3};
        nMinLen = min(nMinLen,iStop-iStart+1);
        nMaxLen = max(nMaxLen,iStop-iStart+1);
    end
    distSumStart = zeros(nMaxLen,1);    % Distances aligned to start of interaction.
    distSumEnd   = zeros(nMaxLen,1);    % Distances aligned to end of interaction.
    
    
    % Process all the interactions.
    filenamePrev = '';
    [m n] = size(interactions);
    for iInteraction=1:m
        filename = interactions{iInteraction,1};
        iStart   = interactions{iInteraction,2};
        iStop    = interactions{iInteraction,3};

        % Read the file if we need to, and compute relative distances.
        if ~strcmp(filename,filenamePrev)
            fprintf ('Reading %s.\n', filename);
            filedata = FlylabReadFile(filename); 
            if (FlylabIsValidFiledata(filedata))
                state2 = FlylabGetObjectState(filedata, 2);
                state3 = FlylabGetObjectState(filedata, 3);
                posRel = state2(:,1:2) - state3(:,1:2);
                distRel = zeros(length(posRel),1);
                for i=1:length(posRel)
                    distRel(i) = sqrt(posRel(i,:)*posRel(i,:)');
                end
            end
        end
        
        % Add the distance from this interaction to the sum.
        offset = nMaxLen-(iStop-iStart+1);
        i=1;
        for iPos=iStart:iStop
            distSumStart(i)      = distSumStart(i)      + distRel(iPos);
            distSumEnd(i+offset) = distSumEnd(i+offset) + distRel(iPos);
            i=i+1;
        end
        
        subplot(2,1,1);
        range = 1:nMinLen;
        plot(distSumStart(range)/iInteraction);
        yl1 = ylim;
        title(sprintf('Avg Distance\nAligned to Start of Chase, nPre=%d, nMinLen=%d, nPost=%d', nPre, nMinLen, nPost));

        subplot(2,1,2);
        range = (length(distSumEnd)-nMinLen+1):length(distSumEnd);
        plot(distSumEnd(range)/iInteraction);
        yl2 = ylim;
        title(sprintf('Avg Distance\nAligned to End of Chase, nPre=%d, nMinLen=%d, nPost=%d', nPre, nMinLen, nPost));

        % Put both plots onto the same scale.
        subplot(2,1,1);
        ylim([0 max(yl1(2),yl2(2))]);
        subplot(2,1,2);
        ylim([0 max(yl1(2),yl2(2))]);

        drawnow;

        filenamePrev = filename;
    end
    
    