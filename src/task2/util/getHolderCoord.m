function [x,y,thetaG] = getHolderCoord(holderNum)
% getHolderCoord  Determine position and orientation to grab a cube depending on the holder
% number
% NOTE: Might need to re-define cube holder positions
%
% Args
% holderNum     : Number of cube holder [1-6]
% 
% Return
% x         : Distance in x direction (mm)
% y         : Distance in y direction (mm)
% theta_g   : Orientation of end effector (radians)
CUBE_SIZE = getOccupancyGridParams().CUBE_DIM;  % 25

switch holderNum
    case 1
        grid = [3,-8];
        thetaG = 0;
    case 2
        grid = [9,0];
        thetaG = 0;
    case 3
        grid = [6,6];
        thetaG = 0;
    case 4
        grid = [5,-5];
        thetaG = -pi/2;
    case 5
        grid = [4,0];
        thetaG = -pi/2;
    case 6
        grid = [0,4];
        thetaG = -pi/2;
    otherwise
        disp("Invalid holder number - only 6 cube holders exist")
        return
end
coord = grid * CUBE_SIZE;
x = coord(1);
y = coord(2);
end