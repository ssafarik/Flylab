function [posTransformed,angleTransformed] = FlylabGetTransformedData(filedata, iFrameParent, iFrameChild)
% Given the .csv data and two frames of reference, return the transformed position and 
% angle of the child frame in the parent frame.
%
% iFrameParent:  The object number in the .csv file (0=arena, 1=robot, 2=fly1, 3=fly2, etc)
% iFrameChild:   The object number in the .csv file (0=arena, 1=robot, 2=fly1, 3=fly2, etc)
%

    % Find which columns to use.
    [m,n] = size(filedata);
    nObjects = (n-2)/6;                 % robot is object 1, flies are 2 through N.
    iColParent = 2+(iFrameParent-1)*6+1;
    iColChild  = 2+(iFrameChild-1)*6+1;
    
    isValidParams = true;
    
    % Get the parent position/angle.
    if iFrameParent<=nObjects
        if iFrameParent==0 % i.e. 'Arena'
            posParent = zeros(m,2)';
            angParent = zeros(m,1);
        else
            posParent = filedata(:,iColParent:iColParent+1)';
            angParent = filedata(:,iColParent+2);
        end;
    else
        isValidParams = false;
        fprintf ('iFrameParent > nObjects.\n');
    end

    % Get the child position/angle.
    if iFrameChild<=nObjects
        if iFrameChild==0 % i.e. 'Arena'
            posChild = zeros(m,2)';
            angChild = zeros(m,1);
        else
            posChild = filedata(:,iColChild:iColChild+1)';
            angChild = filedata(:,iColChild+2);
        end;
    else
        isValidParams = false;
        fprintf ('iFrameChild > nObjects.\n');
    end

    if isValidParams
        posChild = posChild - posParent;
        angChild = angChild - angParent;
        for i=1:m
            R = [cos(-angParent(i)) -sin(-angParent(i)); sin(-angParent(i)) cos(-angParent(i))];
            posChild(:,i) = R*posChild(:,i);
        end
    else
        posChild = zeros(m,2)';
        angChild = zeros(m,1);
    end
    
    posTransformed = posChild';
    angleTransformed = angChild;
