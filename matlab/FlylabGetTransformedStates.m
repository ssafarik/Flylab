function [posTransformed,angleTransformed] = FlylabGetTransformedStates(varargin)
% Given the .csv data and two frames of reference, return the transformed position and 
% angle of the child frame in the parent frame.
%
% iFrameParent:  The object number in the .csv file (0=arena, 1=robot, 2=fly1, 3=fly2, etc)
% iFrameChild:   The object number in the .csv file (0=arena, 1=robot, 2=fly1, 3=fly2, etc)
%

    if nargin==3
        filedata     = varargin{1};
        iFrameParent = varargin{2};
        iFrameChild  = varargin{3};
        iStart       = 1;
        [iStop,n]    = size(filedata.states);
    elseif nargin==5
        filedata     = varargin{1};
        iFrameParent = varargin{2};
        iFrameChild  = varargin{3};
        iStart      = varargin{4};
        iStop       = varargin{5};
    else
        fprintf ('Bad call to FlylabGetTransformedStates().\n');
    end

    isValidParams = true;
    
    % Get the parent position/angle.
    state = FlylabGetObjectState(filedata, iFrameParent);
    if ~isempty(state)
        posParent = state(:, 1:2);
        angParent = state(:, 3);
    else
        isValidParams = false;
        fprintf ('iFrameParent > nObjects.\n');
    end

    % Get the child position/angle.
    state = FlylabGetObjectState(filedata, iFrameChild);
    if ~isempty(state)
        posChild = state(:, 1:2);
        angChild = state(:, 3);
    else
        isValidParams = false;
        fprintf ('iFrameChild > nObjects.\n');
    end

    % Transform the data.
    if isValidParams
        posChild = posChild - posParent;
        angChild = angChild - angParent;
        for i=iStart:iStop
            R = [cos(-angParent(i)) -sin(-angParent(i)); sin(-angParent(i)) cos(-angParent(i))];
            posChild(i,:) = (R*posChild(i,:)')';
        end
    else
        posChild = zeros(iStop-iStart+1, 2);
        angChild = zeros(iStop-iStart+1, 1);
    end
    
    posTransformed = posChild;
    angleTransformed = angChild;
