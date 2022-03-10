function cartesianCoords = OGToCartesianCoords(ogIndices)
    % Converts occupancy grid indices to cartesian coordinates.
    % 
    % Args:
    % ogIndices : (theta_g_idx, theta_1_idx, x_idx, y_idx)
    % 
    % Returns:
    % cartesianCoords : (x, y, z, theta_g)
    
        ogParams = getOccupancyGridParams();
        
        xy_resolution = ogParams.xy_resolution; 
        THETA_G_LIST = ogParams.THETA_G_LIST;
        THETA_1_LIST = ogParams.THETA_1_LIST;
        
        % convert indices to x', y', theta_1, theta_g
        xPrime = (ogIndices(3)-0.5) * xy_resolution;
        yPrime = (ogIndices(4)-0.5) * xy_resolution;
        theta_1 = deg2rad( (ogIndices(2)-0.5)*(THETA_1_LIST(2)-THETA_1_LIST(1)) );
        theta_g = deg2rad( (ogIndices(1)-0.5)*(THETA_G_LIST(2)-THETA_G_LIST(1)) );

        % convert x', y' theta_1 into x, y, z
        
        % Convert coordinates into grid cells
        x = xPrime * cos(theta_1);
        y = xPrime * sin(theta_1);
        z = yPrime;
    
        cartesianCoords = [x, y, z, theta_g];
    
    end