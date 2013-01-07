function scatterPose (x, y, angle, c, r, m)
% scatterPose() is a function much like scatter(), except that it also takes an angle.
% Draws markers at x,y and angle, of the specified radius r, with the given color and marker shape.
%
% x:        List of x coordinates.
% y:        List of y coordinates.
% angle:    List of angles.
% r:        Radius of the markers.  Scalar.
% c:        Color.  A single RGB triple.
% m:        Marker shape.  Can be 'triangle', 'fly', 'robot', or 'o'
% 

    nMarkers = length(x);
    
    % Create 3 by n matrices for shifting each marker to (x,y).
    xRep = repmat(x,3,1);
    yRep = repmat(y,3,1);
    
    % Rotation matrices to get markers at 0 deg == east.
    Rn90 = [cos(-pi/2) -sin(-pi/2);
            sin(-pi/2)  cos(-pi/2)];
    R90 = [cos(pi/2) -sin(pi/2);
           sin(pi/2)  cos(pi/2)];
    R180 = [cos(pi) -sin(pi);
            sin(pi)  cos(pi)];
    
    % Define the marker shape matrix.
    if strcmpi(m,'triangle')
        marker = [-r -r/2;
                   r  0;
                  -r  r/2]';
        xMarker = marker(1,:);
        yMarker = marker(2,:);
        zMarker = ones(1,length(xMarker));
               
    elseif strcmpi(m,'fly')
        flylogo = [-4, 14; 
                    -3, 11; 
                    -7, 7; 
                    -6, 5; 
                    -7, 4; 
                    -9, 4; 
                    -12, 6; 
                    -10, 8; 
                    -12, 10; 
                    -10, 8; 
                    -12, 6; 
                    -9, 4; 
                    -13, 3; 
                    -14, 0; 
                    -11, 1.5; 
                    -9, 4; 
                    -11, 0; 
                    -9, -4; 
                    -11, -1.5; 
                    -14, 0; 
                    -13, -3; 
                    -9, -4; 
                    -12, -6; 
                    -10, -8; 
                    -12, -10; 
                    -10, -8; 
                    -12, -6; 
                    -9, -4; 
                    -7, -4; 
                    -6, -5; 
                    -7, -7; 
                    -3, -11; 
                    -4, -14; 
                    -3, -11; 
                    -7, -7; 
                    -6, -5; 
                    -3, -7; 
                    0,-10; 
                    3, -13; 
                    8,-14.5; 
                    13, -15; 
                    16, -12; 
                    12, -6; 
                    4, -2; 
                    0, -1; 
                    -5, -4; 
                    -2, 0; 
                    -5, 4; 
                    0, 1; 
                    4, 2; 
                    4, -2; 
                    7,0; 
                    4, 2; 
                    12, 6; 
                    16, 12; 
                    13, 15; 
                    8,14.5; 
                    3, 13; 
                    0,10; 
                    -3, 7; 
                    -6, 5; 
                    -7, 7; 
                    -3, 11; 
                    -4, 14]';
        flylogo = r* flylogo / max(max(flylogo')-min(flylogo')); % Scale so max length is r.
        flylogo = R180 * flylogo;
        xMarker = flylogo(1,:);
        yMarker = flylogo(2,:);
        zMarker = ones(1,length(xMarker));
        
    elseif strcmpi(m,'robot') || strcmpi(m,'o')
        % Create a circle for the arena boundary.
        q = (0:pi/r/10:(2*pi));
        xMarker = r * cos(q);
        yMarker = r * sin(q);
        zMarker = ones(1,length(xMarker));
    else
        fprintf ('Unrecognized marker type.\n');
    end
    marker = [xMarker; yMarker; zMarker];
    markersIn = repmat(marker, nMarkers, 1);

    
    % Construct a big transform to move all the markers into position.
    bigT = zeros(nMarkers*3);
    for i=1:nMarkers
        T = [cos(angle(i)), -sin(angle(i)), x(i);
             sin(angle(i)),  cos(angle(i)), y(i);
             0,              0,             1];
        iT = (i-1)*3+1;
        bigT(iT:iT+2,iT:iT+2) = T;
    end
    
    markersOut = bigT * markersIn;
    
    % Need to pull out the (x,y)'s.
    xOut = markersOut( 1:3:nMarkers*3-1,    :)';
    yOut = markersOut((1:3:nMarkers*3-1)+1, :)';
    
    patch (xOut, yOut, c, 'EdgeColor', c*0.7);

    
    
    