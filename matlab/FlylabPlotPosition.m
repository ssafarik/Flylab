function FlylabPlotPosition(filedata, iFrameParent, iFrameChildren, iTrigger, nSubsample)
% FlylabPlotPosition(filedata)
% Plot positions of the specified object (robot=1, fly1=2, fly2=3, etc), in the given frame of reference.
% Draws a circle on the sample given by iTrigger.
%

    % Rotation matrix 90 CCW (so fly is pointing up).
    R90 = [cos(pi/2) -sin(pi/2);
           sin(pi/2)  cos(pi/2)];

    % Create a circle for the arena boundary.
    q = (0:0.01:(2*pi))';
    r = 87;
    xCircle = r * cos(q);
    yCircle = r * sin(q);
    sCircle = 1;
    cCircle = [0 0 0];
    mCircle = '.';
    
    markers = {'o','triangle','triangle','triangle','triangle','triangle','triangle','triangle'};
    colors = [[1 0 0]; [0.3 0.3 0.3]; [0.0 0.8 0.0]; [0.0 0.0 0.8]; [0.0 0.5 0.5]; [0.5 0.0 0.5]; [0.5 0.5 0.0]; [0.3 0.3 0.3]];
    pix = [100 100 100 100 100 100 100 100];
    radii = [0.8 1.25 1.25 1.25 1.25 1.25 1.25 1.25 1.25];
    
    % Start plotting.
    hold off;
    cla;
    axis equal;
    hold on;
    
    for iFrameChild = iFrameChildren
        % Get object pose.
        [pos, ang] = FlylabGetTransformedData(filedata, iFrameParent, iFrameChild);

        % Rotate if necessary so fly points north.
        if iFrameParent~=0
            pos = (R90 * pos')';   
            ang = ang + pi/2;
        end

        % Subsample vectors for scatter plot.
        [m,n] = size(pos);
        iTrigger = max(1,floor(iTrigger/nSubsample));
        x = pos(1:nSubsample:m, 1);
        y = pos(1:nSubsample:m, 2);
        ang=ang(1:nSubsample:m);


        % Remove points outside a boundary.
        if iFrameParent~=0
            rMax = 30;
        else
            rMax = 100;
        end
        iInBounds = find(x>-rMax & x<rMax & y>-rMax & y<rMax);
        x = x(iInBounds);
        y = y(iInBounds);


        % Draw the  positions, with circles on start and trigger.
        if ~isempty(x)
            scatter    (x(1),        y(1),          pix(iFrameChild), colors(iFrameChild,:),                      'o');
            scatter    (x(iTrigger), y(iTrigger), 2*pix(iFrameChild), colors(iFrameChild,:),                      'o');
            scatterPose(x,           y,             ang,              colors(iFrameChild,:), radii(iFrameChild),  markers{iFrameChild});
        end

        if iFrameParent~=0
            axis([-rMax rMax -rMax rMax]);
        end
    end
    
    axis off
    axis equal

    