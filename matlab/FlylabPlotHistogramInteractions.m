function FlylabPlotHistogramInteractions(interactions, radius, nBins)
% FlylabPlotHistogramInteractions(interactions, radius, nBins)
%
% Plot the position histogram over all the given interactions.
% interactions: Cell array M by 3, where each row contains the .csv filename and start/stop indices of the interaction.
% radius:       Radius of the histogram plot.
% nBins:        Number of horiz & vert bins in the histogram.  Also known as the image resolution.
%

    clf;
    
    % Create the histogram.
    histogramLeader = zeros(nBins);
    histogramFollower = zeros(nBins);
    
    % Process all the interactions.
    filenamePrev = '';
    for iInteraction=1:length(interactions)
        filename = interactions{iInteraction,1};
        iStart   = interactions{iInteraction,2};
        iStop    = interactions{iInteraction,3};

        % Read the file if we need to, and compute relative distances.
        if ~strcmp(filename,filenamePrev)
            fprintf ('Reading %s.\n', filename);
            [filedata,iTrigger] = FlylabReadData(filename, -1);
        end
        
        % Add the histogram from this interaction to the sum.
        iFrames = [2 3];
        iFrameLeader = FlylabGetLeader(filedata(iStart:iStop,:), iFrames(1), iFrames(2), radius);
        iFrameFollower = iFrames(find(iFrames~=iFrameLeader));
        histogramLeader = histogramLeader + FlylabGetHistogramPosition(filedata(iStart:iStop,:), iFrameLeader, iFrameFollower, radius, nBins);
        histogramFollower = histogramFollower + FlylabGetHistogramPosition(filedata(iStart:iStop,:), iFrameFollower, iFrameLeader, radius, nBins);
        
        subplot(1,2,1);
        img = (histogramLeader ./ max(max(histogramLeader))) * length(hot);         
        image(img); 
        hold on; 
        [m n]=size(img); scatterPose(n/2, m/2, pi/2, [1 1 1], 10, 'triangle');
        colormap(hot);
        axis equal
        axis([0 nBins 0 nBins]);
        axis xy;
        drawnow;

        subplot(1,2,2);
        img = (histogramFollower ./ max(max(histogramFollower))) * length(hot);         
        image(img); 
        hold on; 
        [m n]=size(img); scatterPose(n/2, m/2, pi/2, [1 1 1], 10, 'triangle');
        colormap(hot);
        axis equal
        axis([0 nBins 0 nBins]);
        axis xy;

        drawnow;

        filenamePrev = filename;
    end
    subplot(1,2,1);
    title('Leader View');
    subplot(1,2,2);
    title('Follower View');
    
    