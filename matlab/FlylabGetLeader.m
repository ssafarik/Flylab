function iFrameLeader=FlylabGetLeader(varargin)
% Determine which of the two objects in the given filedata spends more
% time in front of the other.  Really only makes sense when called on chase data.
%

    if nargin==4
        filedata    = varargin{1};
        iFrameA     = varargin{2};
        iFrameB     = varargin{3};
        radius      = varargin{4};
        iStart      = 1;
        [iStop,n]   = size(filedata.states);
    elseif nargin==6
        filedata    = varargin{1};
        iFrameA     = varargin{2};
        iFrameB     = varargin{3};
        radius      = varargin{4};
        iStart      = varargin{5};
        iStop       = varargin{6};
    else
        fprintf ('Bad call to FlylabGetLeader().\n');
    end
    
    nBins = 256;
    histogram = FlylabGetHistogramPosition(filedata, iFrameA, iFrameB, radius, nBins, iStart, iStop);
    top = sum(sum(histogram((nBins/2+1):nBins,:)));
    bot = sum(sum(histogram(1:nBins/2,:)));
    
    if top>bot
        iFrameLeader = iFrameB;
    else
        iFrameLeader = iFrameA;
    end
    