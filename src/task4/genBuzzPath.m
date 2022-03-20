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
startPos = [8*25, 8*25, 50, 0];
endPos = [8*25, -8*25, 50, 0];
numGrids = ceil(norm(startPos - endPos) / og.CUBE_DIM);
numPoints = numGrids * POINTS_PER_GRID;


buzzPath = linearInterpolate(startPos, endPos, numPoints);

% Determine angle of buzzer based on direction from one path to the next
buzzAngle = [];
for i=1:size(buzzPath,1)-1
    a = buzzPath(i,:)
    b = buzzPath(i+1,:)
    buzzAngle(i) = atan2(norm(cross(a,b)), dot(a,b))
end

end