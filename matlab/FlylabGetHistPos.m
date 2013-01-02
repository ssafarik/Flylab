function histFile = FlylabGetHistPos(filedata, radius, nBins, frameParent, frameChild)
% FlylabGetHistPos(filedata, nBins, nSaturation)
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
        [posFile,angleFile] = FlylabGetTransformedData(filedata, frameParent, frameChild);

        % Append two points at corners [-radius -radius]; [+radius +radius].  Clip points outside that rect.
        posFile = [posFile; [-radius -radius]; [+radius +radius]];
        posFile = clipmat(posFile, -radius, radius);

        % Get the histogram.
        histFile = hist3(posFile,[nBins nBins]);

        % Remove the two corner points we appended earlier.
        histFile(1,1) = histFile(1,1)-1;
        histFile(nBins,nBins) = histFile(nBins,nBins)-1;
    else
        histFile = zeros(nBins,nBins);
    end
    
    % Zero out the edges where we clipped.
    histFile(1,:) = 0;  % top row
    histFile(:,1) = 0;  % left col
    histFile(nBins,:) = 0;  % bottom row
    histFile(:,nBins) = 0;  % right col
    
    % Rotate 90 twice.
    %histFile = rot90(histFile,2);
    
    