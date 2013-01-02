function [posTransformed,angleTransformed] = FlylabGetTransformedData(filedata, frameParent, frameChild)
% Given the .csv data and two frames of reference, return the transformed position and 
% angle of the child frame in the parent frame.
%

    [m,n] = size(filedata);
    if n>=14
        switch lower(frameParent)
            case 'arena'
                posParent = zeros(m,2)';
                angParent = zeros(m,1);
            case 'robot'
                posParent = filedata(:,3:4)';
                angParent = filedata(:,5);
            case 'fly'
                posParent = filedata(:,9:10)';
                angParent = filedata(:,11);
            otherwise
                posParent = zeros(m,2)';
                angParent = zeros(m,1);
                fprintf ('Unknown frame: %s.', frame);
        end

        switch lower(frameChild)
            case 'arena'
                posChild = zeros(m,2)';
                angChild = zeros(m,1);
            case 'robot'
                posChild = filedata(:,3:4)';
                angChild = filedata(:,5);
            case 'fly'
                posChild = filedata(:,9:10)';
                angChild = filedata(:,11);
            otherwise
                posChild = zeros(m,2)';
                angChild = zeros(m,1);
                fprintf ('Unknown frame: %s.', frame);
        end


        posChild = posChild - posParent;
        for i=1:m
            R = [cos(-angParent(i)) -sin(-angParent(i)); sin(-angParent(i)) cos(-angParent(i))];
            posChild(:,i) = R*posChild(:,i);
            angChild = angChild - angParent;
        end

        posTransformed = posChild';
        angleTransformed = angChild;
    else
        posTransformed = [];
        angleTransformed = [];
    end        
        
