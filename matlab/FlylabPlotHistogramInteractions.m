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
    [m n] = size(interactions);
    for iInteraction=1:m
        filename = interactions{iInteraction,1};
        iStart   = interactions{iInteraction,2};
        iStop    = interactions{iInteraction,3};

        % Read the file if we need to, and compute relative distances.
        if ~strcmp(filename,filenamePrev)
            fprintf ('Reading %s.\n', filename);
            filedata = FlylabReadFile(filename);
        end
        
        % Add the histogram from this interaction to the sum.
        iFrames = [2 3];
        iFrameLeader = FlylabGetLeader(filedata, iFrames(1), iFrames(2), radius, iStart, iStop);
        iFrameFollower = iFrames(find(iFrames~=iFrameLeader));
        histogramLeader   = histogramLeader   + FlylabGetHistogramPosition(filedata, iFrameLeader, iFrameFollower, radius, nBins, iStart, iStop);
        histogramFollower = histogramFollower + FlylabGetHistogramPosition(filedata, iFrameFollower, iFrameLeader, radius, nBins, iStart, iStop);
        
        subplot(1,2,1);
        img = (histogramLeader ./ max(max(histogramLeader))) * length(hot);         
        image(img); 
        hold on; 
        [m n]=size(img); scatterPose(n/2, m/2, pi/2, 10, [1 1 1], 'triangle');
        colormap(hot);
        axis equal
        axis([0 nBins 0 nBins]);
        axis xy;
        drawnow;

        subplot(1,2,2);
        img = (histogramFollower ./ max(max(histogramFollower))) * length(hot);         
        image(img); 
        hold on; 
        [m n]=size(img); scatterPose(n/2, m/2, pi/2, 10, [1 1 1], 'triangle');
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
    
    
