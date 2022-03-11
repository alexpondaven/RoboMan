function ogIndices = cartesianToOGCoords(cartesianCoords)
% Converts cartesian coordinates (x, y, z, theta_g) to 
% an appropriate index of the occupancy grid.
% 
% Args:
% cartesianCoords : (x, y, z, theta_g)
% 
% Returns:
% ogIndices : (theta_g_idx, theta_1_idx, x_idx, y_idx)

    % Get occupancy grid parameters
    ogParams = getOccupancyGridParams();
    
    xy_resolution = ogParams.xy_resolution;        % resolution for each cube coordinate
    THETA_G_LIST = ogParams.THETA_G_LIST;
    THETA_1_LIST = ogParams.THETA_1_LIST;
    X_LIST = ogParams.X_LIST;
    Y_LIST = ogParams.Y_LIST;
    
    % convert cartesian coordinates into x', y' theta_1
    xPrime = hypot( cartesianCoords(1), cartesianCoords(2) );
    yPrime = cartesianCoords(3);
    theta_1 = rad2deg( atan2( cartesianCoords(2), cartesianCoords(1) ));
    theta_g = rad2deg( cartesianCoords(4) );

    % fprintf("[cartesianToOGCoords] x': %0.2f y': %0.2f, t_1: %0.2f, t_g: %0.2f\n", xPrime, yPrime, theta_1, theta_g);
    
    % Convert coordinates into grid cells
    x_idx = 1 + round( (xPrime - X_LIST(1)) / xy_resolution );    % X_LIST(1) is the starting value of X_LIST
    y_idx = 1 + round( (yPrime - Y_LIST(1)) / xy_resolution );    % Y_LIST(1) is the starting value of Y_LIST
    % Resolution for theta_1 and theta_g is the distance in the array.
    theta_1_idx = 1 + round( (theta_1-THETA_1_LIST(1)) / (THETA_1_LIST(2)-THETA_1_LIST(1)) );
    theta_g_idx = 1 + round( (theta_g-THETA_G_LIST(1)) / (THETA_G_LIST(2)-THETA_G_LIST(1)) );

    ogIndices = [theta_g_idx, theta_1_idx, x_idx, y_idx];

end