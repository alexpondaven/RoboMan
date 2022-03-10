function waypoints = AstarSearch( startPos, endPos, occupancyGrid )
% Finds the shortest collision-free path from startPos to endPos through
% the occupancy grid.
% 
% Args:
% startPos      : The starting position (x, y, z, theta_g) of the end effector.
% endPos        : The goal position (x, y, z, theta_g) of the end effector.
% occupancyGrid : 4D array of (theta_g, theta_1, x', y') configurations
% 
% Returns:
% waypoints : 2D array of (x, y, z, theta_g) angles.

ogParams = getOccupancyGridParams();

%% Convert start and end positions to grid coordinates
startIdx = cartesianToOGCoords(startPos);
goalIdx = cartesianToOGCoords(endPos);

%% Perform A* Search
% TODO


%% Convert grid coordinates found in goal to cartesian coordinates
waypoints = zeros(4, size(waypointIdx,2));
for idx=1:size(waypointIdx,2)
    waypoints(:,idx) = OGToCartesianCoords(waypointIdx(:,idx));
end

end