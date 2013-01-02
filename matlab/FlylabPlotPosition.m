function FlylabPlotPosition(filedata, frameParent, iTrigger, nSubsample)
% FlylabPlotPosition(filedata)
% Plot positions of the fly & robot, in the given frame of reference.
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
    
    % Get robot & fly poses.
    [posRobot, aRobot] = FlylabGetTransformedData(filedata, frameParent, 'Robot');
    [posFly, aFly] = FlylabGetTransformedData(filedata, frameParent, 'Fly');
    aFly = aFly - pi/2;

    % Rotate if necessary so fly points north.
    if ~strcmpi(frameParent,'Arena')
        posRobot = (R90 * posRobot')';   
        aRobot = aRobot + pi/2;
        posFly = (R90 * posFly')';   
        aFly = 0*aFly;
    end

    % Set up vectors for scatter plot.
    [m,n] = size(posRobot);
    iTrigger = max(1,floor(iTrigger/nSubsample));
    
    xRobot=posRobot(1:nSubsample:m, 1);
    yRobot=posRobot(1:nSubsample:m, 2);
    aRobot=aRobot(1:nSubsample:m);
    pixRobot = 10;
    rRobot = 0.8;    % radius
    cRobot = [1 0 0];
    mRobot = 'o';
    xFly=posFly(1:nSubsample:m, 1);
    yFly=posFly(1:nSubsample:m, 2);
    aFly=aFly(1:nSubsample:m);
    pixFly = 10;
    rFly = 1.25;       % radius
    cFly = [0.3 0.3 0.3];
    mFly = 'triangle';

    % Start plotting.
    hold off;
    cla;
    axis equal;
    
    
    % Remove points outside a boundary.
    rMax = 20;
    if ~strcmpi(frameParent,'Arena')
        iInBounds = find(xRobot>-rMax & xRobot<rMax & yRobot>-rMax & yRobot<rMax);
        xRobot = xRobot(iInBounds);
        yRobot = yRobot(iInBounds);
    end
    
    % Draw the arena bounds if in Arena frame.
    %if strcmpi(frameParent,'Arena')
    %    scatter(xCircle, yCircle, sCircle, cCircle, mCircle);
    %end
    %hold on;
    
    % Draw the Robot positions, with circle on start.
    if ~isempty(xRobot)
        hold on;
        scatter(xRobot(1),        yRobot(1),        10*pixRobot, cRobot,           mRobot);
        scatterPose(xRobot,       yRobot,           aRobot,      cRobot,  rRobot,  mRobot);
    end
    
    % Draw the Fly positions, with circle on start.
    if ~isempty(xFly)
        scatterPose(xFly,         yFly,             aFly,         cFly,   rFly,    mFly);
    end
    
    if strcmpi(frameParent,'Arena')
        scatter(xRobot(iTrigger), yRobot(iTrigger), 20*pixRobot,  cRobot,          mRobot);
        scatter(xFly(1),          yFly(1),          10*pixFly,    cFly,            mRobot);
        scatter(xFly(iTrigger),   yFly(iTrigger),   20*pixFly,    cFly,            mRobot);
    else
        axis([-rMax rMax -rMax rMax]);
    end

    axis off
    axis equal

    