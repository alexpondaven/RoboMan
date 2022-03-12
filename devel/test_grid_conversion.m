% Debug script for converting back and forth between coordinate systems

%% Function parameters
ogParams = getOccupancyGridParams();

THETA_G_LIST = ogParams.THETA_G_LIST;
THETA_1_LIST = ogParams.THETA_1_LIST;
X_LIST = ogParams.X_LIST;
Y_LIST = ogParams.Y_LIST;

coords = [19,0,0,0]
og_coords = cartesianToOGCoords(coords)
fprintf("t_g: %d | t_1: %d | x': %0.1f | y': %0.1f\n", ...
    THETA_G_LIST(og_coords(1)), ...
    THETA_1_LIST(og_coords(2)), ...
    X_LIST(og_coords(3)), ...
    Y_LIST(og_coords(4)) );

OGToCartesianCoords(og_coords)