function state = FlylabGetObjectState(filedata, iFrame)
% Return the (x,y,a,vx,vy,va) state vectors of the specified frame from the given data.
%

    % Find which columns to use.
    [m,n] = size(filedata);
    nObjects = (n-2)/6;                 % robot is object 1, flies are 2 through N.
    iCol = 2+(iFrame-1)*6+1;
    
    % Get the parent state.
    if iFrame<=nObjects
        if iFrame==0 % i.e. 'Arena'
            state = zeros(m,6); % [x,y,a,vx,vy,va]
        else
            state = filedata(:,iCol:iCol+5);
        end;
    else
        state = [];
    end

