function state = FlylabGetObjectState(varargin)
% Return the (x,y,a,vx,vy,va) state vectors of the specified frame from the given data.
%

    if nargin==2
        filedata     = varargin{1};
        iFrame       = varargin{2};
        iStart       = 1;
        [iStop,n]    = size(filedata.states);
     elseif nargin==4
         filedata     = varargin{1};
         iFrame       = varargin{2};
         iStart       = varargin{3};
         iStop        = varargin{4};
         [m,n]        = size(filedata.states);
    else
        fprintf ('Bad call to FlylabGetObjectState().\n');
    end
    
    if str2num(filedata.header.version.versionFile) < 2.7
        nCols = 6;
    else
        nCols = 8;
    end
    
    
    % Find which columns to use.
    nObjects = (n-2)/nCols;                 % robot is object 1, flies are 2 through N.
    iCol = 2+(iFrame-1)*nCols+1;
    
    % Get the requested states.
    if iFrame<=nObjects
        if iFrame==0 % i.e. 'Arena'
            state = zeros(iStop-iStart+1, nCols); % [x,y,a,vx,vy,va]
        else
            state = filedata.states(:, iCol:iCol+nCols-1);
        end;
    else
        state = [];
    end

    
    % Add columns to fill out missing wing angles.
    [m,n] = size(state);
    nMissing = max(0, 8-n);
    state = horzcat(state, zeros(m, nMissing));
    