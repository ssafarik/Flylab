function iFrameLeader=FlylabGetLeader(filedata, iFrameA, iFrameB, radius)
% Determine which of the two objects in the given filedata spends more
% time in front of the other.  Really only makes sense when called on chase data.
%
    
    nBins = 256;
    histogram = FlylabGetHistogramPosition(filedata, iFrameA, iFrameB, radius, nBins);
    top = sum(sum(histogram((nBins/2+1):nBins,:)));
    bot = sum(sum(histogram(1:nBins/2,:)));
    
    if top>bot
        iFrameLeader = iFrameB;
    else
        iFrameLeader = iFrameA;
    end
    