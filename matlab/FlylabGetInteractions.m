function [start,stop] = FlylabGetInteractions(filedata, iFrameParent, iFrameChild, nMinLen, nPre, nPost, criteria)
% [start,stop] = FlylabGetInteractions(filedata, iFrameParent, iFrameChild, nMinLen)
% Find the set of interaction sequences in the given data.
% Each interaction sequence is defined by a start and stop index, 
% and sequence i is given by the indices start(i) and stop(i).
% 
% start:    List of start-of-sequence indices.
% stop:     List of end-of-sequence indices.
% nMinLen:  Minimum length of a valid interaction sequence.
    
    [m,n] = size(filedata);
    isValidParams = true;
    
    % Get the parent position/angle.
    stateP = FlylabGetObjectState(filedata, iFrameParent);
    if isempty(stateP)
        isValidParams = false;
        fprintf ('iFrameParent > nObjects.\n');
    end

    % Get the child position/angle.
    stateC = FlylabGetObjectState(filedata, iFrameChild);
    if isempty(stateC)
        isValidParams = false;
        fprintf ('iFrameChild > nObjects.\n');
    end

    start = [];
    stop  = [];
    if isValidParams
        iCount = 0;
        bPrev = false;
        for iPos=1:m
            b = MeetsCriteria(stateP(iPos,:), stateC(iPos,:), criteria) && (iPos<m);
            
            % Rising edge: save start position.
            if b && ~bPrev
                iStart = max(0,iPos-nPre);
            end
            
            % Duration of the interaction.
            if b
                iCount = iCount+1;
            end
            
            % Falling edge: save stop position.
            if ~b && bPrev
                if iCount>=nMinLen
                    iStop = min(iPos+nPost,m);
                    start = [start iStart];
                    stop  = [stop iStop];
                end
            end
            
            if ~b
                iCount = 0;
            end
            bPrev = b;
        end
    end
    
end

function b=MeetsCriteria(stateP, stateC, criteria)
    posParent = stateP(1:2);
    angParent = stateP(3);
    velParent = stateP(4:5);
    vaParent  = stateP(6);

    posChild = stateC(1:2);
    angChild = stateC(3);
    velChild = stateC(4:5);
    vaChild  = stateC(6);

    % Relative position & velocity.
    pos = posParent-posChild;
    vel = velParent-velChild;
    
    speedRel = sqrt(vel * vel');
    speedP = sqrt(velParent * velParent');
    speedC = sqrt(velChild * velChild');
    dist = sqrt(pos * pos');
    angleRel = mod((angParent - angChild),2*pi);
    
    switch criteria
        case 'chase'
            b=CriteriaChase(dist, angleRel, speedP, speedC, speedRel);
        case 'proximity'
            b=CriteriaProximity(dist, angleRel, speedP, speedC, speedRel);
        case 'fight'
            b=CriteriaFight(dist, angleRel, speedP, speedC, speedRel);
        case 'all'
            b=CriteriaAll(dist, angleRel, speedP, speedC, speedRel);
        otherwise
            b=false;
    end
end

function b=CriteriaChase(dist, angleRel, speedP, speedC, speedRel)
    if (dist<10) && (angleRel<0.75 || angleRel>(2*pi-0.75)) && (speedP>2 && speedC>2)
        b = true;
    else
        b = false;
    end
end

function b=CriteriaProximity(dist, angleRel, speedP, speedC, speedRel)
    if (dist<10)
        b = true;
    else
        b = false;
    end
end

function b=CriteriaFight(dist, angleRel, speedP, speedC, speedRel)
    % Not yet implemented.
    b = false;
end

function b=CriteriaAll(dist, angleRel, speedP, speedC, speedRel)
    b = true;
end
    