function [buzzPath, buzzAngle] = genBuzzPath()
% GenBuzzPath Generates the path or waypoints that the buzzer must follow to complete the buzzer game
% Challenge - Generate this based on an image?
%
% Returns:
% buzzPath      : Set of waypoints [x,y,z,thetaG] that the arm must follow
% buzzAngle : Buzzer angle at each waypoints
POINTS_PER_GRID = 10;

% Basic:
% Straight line
ogParams = getOccupancyGridParams();

% coords = [  8 8 2 0
%             8 0 2 0
%             3 0 2 0 ] * ogParams.CUBE_DIM;  % grid positions
coords = [  8 8 5 0
            8 0 5 0 ] * ogParams.CUBE_DIM;  % grid positions

% startPos = [8*25, 8*25, 50, 0];
% endPos = [8*25, -8*25, 50, 0];


buzzPath = [];
for i=1:size(coords,1)-1
    startPos = coords(i,:);
    endPos = coords(i+1,:);
    numGrids = ceil(norm(startPos - endPos) / ogParams.CUBE_DIM);
    numPoints = numGrids * POINTS_PER_GRID;
    buzzPath = [buzzPath; linearInterpolate(startPos, endPos, numPoints)];
end

% Determine angle of buzzer based on direction from one path to the next
buzzAngle = [];
for i=1:size(buzzPath,1)-1
    a = buzzPath(i,1:2);
    b = buzzPath(i+1,1:2);
    dv = b-a;
    buzzAngle(i) = atan2(dv(2), dv(1));
end

buzzAngle(end+1) = buzzAngle(end);  % Pad to get the same dimensions as vias

end