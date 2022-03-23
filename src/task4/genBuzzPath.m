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
            8 5 5 0
            6 3 5 0
            6 0 5 0 ] * ogParams.CUBE_DIM;  % grid positions

% Simpler
z = 115 / ogParams.CUBE_DIM; % Easy way to set in mm
coords = [  8 - (10)/ogParams.CUBE_DIM  8 z + (0)/ogParams.CUBE_DIM 0
            8 - (10)/ogParams.CUBE_DIM  7 z + (0)/ogParams.CUBE_DIM 0
            7 - (10)/ogParams.CUBE_DIM  6 z + (0)/ogParams.CUBE_DIM 0
            8 - (10)/ogParams.CUBE_DIM  5 z + (0)/ogParams.CUBE_DIM 0
            7 - (0)/ogParams.CUBE_DIM  3 z + (0)/ogParams.CUBE_DIM 0
            7 - (0)/ogParams.CUBE_DIM  2.5 z + (0)/ogParams.CUBE_DIM 0] * ogParams.CUBE_DIM;  % grid positions

coords = [coords ; flip(coords,1)];

% startPos = [8*25, 8*25, 50, 0];
% endPos = [8*25, -8*25, 50, 0];


buzzPath = [];
for i=1:size(coords,1)-1
    startPos = coords(i,:);
    endPos = coords(i+1,:);
    numGrids = ceil(norm(startPos - endPos) / ogParams.CUBE_DIM);
    numPoints = numGrids * POINTS_PER_GRID;
    buzzAdd = linearInterpolate(startPos, endPos, numPoints);
    buzzPath = [buzzPath; buzzAdd(1:end-1,:)];
end
buzzPath(end, :) = coords(end, :);  % ensure the last item is still added

% Determine angle of buzzer based on direction from one path to the next
buzzAngle = [];
for i=1:size(buzzPath,1)-1
    a = buzzPath(i,1:2);
    b = buzzPath(i+1,1:2);
    dv = b-a;
    buzzAngle(i) = atan2(dv(2), dv(1));
    if buzzAngle(i) > 0
        buzzAngle(i) = buzzAngle(i) - pi;  % hack to keep it within servo limits
    end
end

% Smooth out motion of angle theta5
k = 11;
movingAvgFilter = ones(1,k)/k;
offset = 1;                % lookahead
pad = floor(k/2);
buzzAngle = conv(buzzAngle(:), movingAvgFilter);
% figure
% plot(buzzAngle);
% hold on
buzzAngle = buzzAngle(pad-offset+1:end-pad-offset);
% plot(buzzAngle)

buzzAngle(end+1) = buzzAngle(end);  % Pad to get the same dimensions as vias

end