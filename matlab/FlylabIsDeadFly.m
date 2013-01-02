function isDead = FlylabIsDeadFly(filename)
% isDead = FlylabIsDeadFly(filename)
% Determine if the fly in the given file is dead.
%
    tol = 9;    % Tolerance for fly movement.  Millimeters.
    
    [filedata,iTrigger] = FlylabReadData(filename, -1);
    isDead = true;

    [m n] = size(filedata);
    datarange = ceil((m/3)):m;              % If fly was dead for the last 2/3rds of the data, then it counts as dead.
    
    if length(datarange)>=2 && n>=10         % Must have at least two samples to detect a living fly.
        xMin = min(filedata(datarange,9));
        xMax = max(filedata(datarange,9));
        yMin = min(filedata(datarange,10));
        yMax = max(filedata(datarange,10));

        if (xMax-xMin > tol) || (yMax-yMin > tol)
            isDead = false;
        end
    end
    