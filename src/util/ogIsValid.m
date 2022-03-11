function ret = ogIsValid(ogCoords)
% Checks if the given occupancy grid coordinates are valid.

    ogParams = getOccupancyGridParams();
    N_X_LIST = size(ogParams.X_LIST, 2);
    N_Y_LIST = size(ogParams.Y_LIST, 2);
    N_THETA_G_LIST = size(ogParams.THETA_G_LIST, 2);
    N_THETA_1_LIST = size(ogParams.THETA_1_LIST, 2);

    if  ogCoords(1) > 0 && ogCoords(1) <= N_THETA_G_LIST && ...
        ogCoords(2) > 0 && ogCoords(2) <= N_THETA_1_LIST && ...   % bounds checking
        ogCoords(3) > 0 && ogCoords(3) <= N_X_LIST && ...
        ogCoords(4) > 0 && ogCoords(4) <= N_Y_LIST
        ret = true;
    else 
        ret = false;
    end

end