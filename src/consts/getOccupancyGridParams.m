function ogParams = getOccupancyGridParams()
% Returns user-defined parameters for occupancy grid.

    ogParams.XY_SCALING = 25;           % Each grid square is 25mm
    ogParams.CUBE_DIM = 25;             % in millimeters
    ogParams.HOLDER_DIM = 30;
    ogParams.HOLDER_HEIGHT = 20;        % Top of the cube holder
    ogParams.HOVER_HEIGHT = 20;         % How high above cube stack to place end effector before pick/place
    ogParams.xy_resolution = 12.5;

    % ogParams.THETA_G_LIST = -180:22.5:0;  % Angle to step through from -180 (pointing backwards) to 0 (pointing forwards)
    ogParams.THETA_G_LIST = -90:10:0;  % Angle to step through from -180 (pointing backwards) to 0 (pointing forwards)
    ogParams.THETA_1_LIST = -90:5:90;  % Left and right limits
    ogParams.X_LIST = 0:ogParams.xy_resolution:250;
    ogParams.Y_LIST = 0:ogParams.xy_resolution:250;

end

