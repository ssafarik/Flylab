function matOut = FlylabClipByCount(matIn, nClipped)
% Clip a matrix to a value such that the given numbers of entries are clipped.
% nClipped:  Number of entries that are clipped in the return value.

    % Find a clip value that gives the specified number of entries.
    clipval = 1;
    while sum(sum(matIn>clipval))>nClipped
        clipval = clipval*1.1;
    end
    
    % Clip matrix to a limit.
    matOut = clipmat(matIn, 0, clipval);          
