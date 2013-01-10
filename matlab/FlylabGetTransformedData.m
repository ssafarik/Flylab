function [posTransformed,angleTransformed] = FlylabGetTransformedData(filedata, iFrameParent, iFrameChild)
% Given the .csv data and two frames of reference, return the transformed position and 
% angle of the child frame in the parent frame.
%
% iFrameParent:  The object number in the .csv file (0=arena, 1=robot, 2=fly1, 3=fly2, etc)
% iFrameChild:   The object number in the .csv file (0=arena, 1=robot, 2=fly1, 3=fly2, etc)
%

    [m,n] = size(filedata);
    isValidParams = true;
    
    % Get the parent position/angle.
    state = FlylabGetObjectState(filedata, iFrameParent);
    if ~isempty(state)
        posParent = state(:,1:2);
        angParent = state(:,3);
    else
        isValidParams = false;
        fprintf ('iFrameParent > nObjects.\n');
    end

    % Get the child position/angle.
    state = FlylabGetObjectState(filedata, iFrameChild);
    if ~isempty(state)
        posChild = state(:,1:2);
        angChild = state(:,3);
    else
        isValidParams = false;
        fprintf ('iFrameChild > nObjects.\n');
    end

    if isValidParams
        posChild = posChild - posParent;
        angChild = angChild - angParent;
        for i=1:m
            R = [cos(-angParent(i)) -sin(-angParent(i)); sin(-angParent(i)) cos(-angParent(i))];
            posChild(i,:) = (R*posChild(i,:)')';
        end
    else
        posChild = zeros(m,2);
        angChild = zeros(m,1);
    end
    
    posTransformed = posChild;
    angleTransformed = angChild;
