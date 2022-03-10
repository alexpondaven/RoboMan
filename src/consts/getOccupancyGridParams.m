function ogParams = getOccupancyGridParams()
% Returns user-defined parameters for occupancy grid.

    xy_resolution = 5;                 % resolution for each cube coordinate
    ogParams.xy_resolution = xy_resolution;
    ogParams.XY_SCALING = 25;           % Each grid square is 25mm
    ogParams.CUBE_DIM = 25;             % in millimeters
    ogParams.HOLDER_DIM = 30;
    ogParams.HOLDER_HEIGHT = 15;        % Center of the cube when it is placed on a holder

    ogParams.THETA_G_LIST = -180:10:0;  % Angle to step through from -180 (pointing backwards) to 0 (pointing forwards)
    ogParams.THETA_1_LIST = -91:10:91;  % Left and right limits
    ogParams.X_LIST = 0:xy_resolution:250;
    ogParams.Y_LIST = 0:xy_resolution:250;
end

