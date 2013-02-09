function isDead = FlylabHasDeadFly(filedata)
% isDead = FlylabHasDeadFly(filedata)
% Determine if any of the flies in the given filedata are dead.
%

    if nargin==1
        filedata     = varargin{1};
        iStart       = 1;
        [iStop,n]    = size(filedata.states);
    elseif nargin==3
        filedata     = varargin{1};
        iStart       = varargin{2};
        iStop        = varargin{3};
    else
        fprintf ('Bad call to FlylabHasDeadFly().\n');
    end

    tol = 9;    % Tolerance for fly movement.  Millimeters.
    
    datarange = iStart:iStop;
    nObjects = (n-2)/6;                 % robot is object 1, flies are 2 through N.
    isDeadFlies = ones(nObjects, 1);    % An array of flags.
    
    for iFly=2:nObjects
        iCol = 2+(iFly-1)*6+1;
    
        if length(datarange)>=2         % Must have at least two samples to detect a living fly.
            xMin = min(filedata.states(datarange,iCol));
            xMax = max(filedata.states(datarange,iCol));
            yMin = min(filedata.states(datarange,iCol+1));
            yMax = max(filedata.states(datarange,iCol+1));

            if (xMax-xMin > tol) || (yMax-yMin > tol)
                isDeadFlies(iFly) = 0;
            end
        end
    end    
    
    isDead = sum(isDeadFlies(2:nObjects)) > 0;
    