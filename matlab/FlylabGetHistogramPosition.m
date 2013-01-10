function histogram = FlylabGetHistogramPosition(filedata, iFrameParent, iFrameChild, radius, nBins)
% FlylabGetHistogramPosition(filedata, iFrameParent, iFrameChild, radius, nBins)
% Get the position histogram of the child frame in the parent frame.
%
% filedata:     Data from the Flylab .csv file.
% radius:       Size of the arena.  Used for clipping bogus locations.
% nBins:        Number of histogram bins x and y.
% frameParent:  The parent frame.
% frameChild:   The child frame.
%

    [m,n] = size(filedata);
    if n>=14
        [pos,angle] = FlylabGetTransformedData(filedata, iFrameParent, iFrameChild);

        % Append two points at corners [-radius -radius]; [+radius +radius].  Clip points outside that rect.
        pos = [pos; [-radius -radius]; [+radius +radius]];
        pos = clipmat(pos, -radius, radius);

        % Get the histogram.
        histogram = hist3(pos,[nBins nBins]);

        % Remove the two corner points we appended earlier.
        histogram(1,1) = histogram(1,1)-1;
        histogram(nBins,nBins) = histogram(nBins,nBins)-1;
    else
        histogram = zeros(nBins,nBins);
    end
    
    % Zero out the edges where we clipped.
    histogram(1,:) = 0;  % top row
    histogram(:,1) = 0;  % left col
    histogram(nBins,:) = 0;  % bottom row
    histogram(:,nBins) = 0;  % right col
    
    
    