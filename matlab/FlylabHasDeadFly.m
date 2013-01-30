function isDead = FlylabHasDeadFly(filedata)
% isDead = FlylabHasDeadFly(filedata)
% Determine if any of the flies in the given filedata are dead.
%
    tol = 9;    % Tolerance for fly movement.  Millimeters.
    
    [m n] = size(filedata);
    datarange = 1:m;%ceil((m/3)):m;          % If fly was dead for the last 2/3rds of the data, then it counts as dead.
    nObjects = (n-2)/6;                 % robot is object 1, flies are 2 through N.
    isDeadFlies = ones(nObjects, 1);    % An array of flags.
    
    for iFly=2:nObjects
        iCol = 2+(iFly-1)*6+1;
    
        if length(datarange)>=2         % Must have at least two samples to detect a living fly.
            xMin = min(filedata(datarange,iCol));
            xMax = max(filedata(datarange,iCol));
            yMin = min(filedata(datarange,iCol+1));
            yMax = max(filedata(datarange,iCol+1));

            if (xMax-xMin > tol) || (yMax-yMin > tol)
                isDeadFlies(iFly) = 0;
            end
        end
    end    
    
    isDead = sum(isDeadFlies(2:nObjects)) > 0;
    