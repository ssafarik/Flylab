function statesTransformed = FlylabGetTransformedStates(varargin)
%function [posTransformed,angleTransformed] = FlylabGetTransformedStates(varargin)
% Given the .csv data and two frames of reference, return the transformed
% state of the child frame in the parent frame.
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
    else
        fprintf ('Bad call to FlylabGetTransformedStates().\n');
    end

    isValidParams = true;
    
    % Get the parent position/angle.
    stateParent = FlylabGetObjectState(filedata, iFrameParent);
    if isempty(stateParent)
        isValidParams = false;
        fprintf ('iFrameParent > nObjects.\n');
    end

    % Get the child position/angle.
    stateChild = FlylabGetObjectState(filedata, iFrameChild);
    if isempty(stateChild)
        isValidParams = false;
        fprintf ('iFrameChild > nObjects.\n');
    end

    % Transform the data.
    statesTransformed = zeros(iStop-iStart+1, 8);
    if isValidParams
        for i=iStart:iStop
            xP  = stateParent(i,1);
            yP  = stateParent(i,2);
            aP  = stateParent(i,3);
            vxP = stateParent(i,4);
            vyP = stateParent(i,5);
            vaP = stateParent(i,6);
            w1P = stateParent(i,7);
            w2P = stateParent(i,8);
            vecP = [xP, yP, 1, aP, vxP, vyP, 1, vaP];

            xC  = stateChild(i,1);
            yC  = stateChild(i,2);
            aC  = stateChild(i,3);
            vxC = stateChild(i,4);
            vyC = stateChild(i,5);
            vaC = stateChild(i,6);
            w1C = stateChild(i,7);
            w2C = stateChild(i,8);
            vecC = [xC, yC, 1, aC, vxC, vyC, 1, vaC];

            % Untranslate by parent position.
            pPc = [1, 0, -xP;
                   0, 1, -yP;
                   0, 0, 1];
            % Unrotate by parent rotation.
            pRc = [cos(-aP), -sin(-aP),   0;
                   sin(-aP),  cos(-aP),   0;
                   0,         0,          1];

            % Transform the pose.
            pOut = pRc * pPc * [xC; yC; 1];
            aOut = aC - aP;

            % Transform the vel.
            pVc = [vxC; vyC; 1] - pRc*[vxP; vyP; 1];
            vxOut = pVc(1);
            vyOut = pVc(2);
            vaOut = vaC - vaP;
               
            %          vecChild                                   vecParent
            %            x          y     1    a   vx vy  1   va    x  y  1   a   vx  vy   1  va
%             T = [cos(-aP), -sin(-aP),   0,   0,   0, 0, 0,   0,   0, 0, -xP,  0,   0,  0,  0,  0; %  xT
%                  sin(-aP),  cos(-aP),   0,   0,   0, 0, 0,   0,   0, 0, -yP,  0,   0,  0,  0,  0; %  yT
%                  0,         0,          1,   0,   0, 0, 0,   0,   0, 0, 0,  0,   0,  0,  0,  0; %   1
%                  
%                  0,         0,          0,   1,   0, 0, 0,   0,   0, 0, 0, -1,   0,  0,  0,  0; %  aT
%                  
%                  0,         0,          0,   0,   1, 0, 0,   0,   0, 0, 0,  0,  -1,  0,  0,  0; % vxT
%                  0,         0,          0,   0,   0, 1, 0,   0,   0, 0, 0,  0,   0, -1,  0,  0; % vyT
%                  0,         0,          0,   0,   0, 0, 1,   0,   0, 0, 0,  0,   0,  0,  1,  0; %   1
%                  
%                  0,         0,          0,   0,   0, 0, 0,   1,   0, 0, 0,  0,   0,  0,  0, -1];% vaT
%             vecIn = [vecC, vecP];
%             vecOut = (T*vecIn')';
            statesTransformed(i,:) = [pOut(1), pOut(2), aOut, vxOut, vyOut, vaOut, w1C, w2C];
        end
    end
    
