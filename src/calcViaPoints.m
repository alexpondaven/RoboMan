function via_paths = calcViaPoints(AStarWaypoints, occupancyGridVect)
% Calculates a set of via points from one A* waypoint to the next.
% Args:
% AStarWaypoints     : Set of waypoints (x,y,z,theta_g) that end effector has to move through
% occupancyGridVect  : Vector of occupancy grids (length 1 less than AStarWaypoints) 
%                      that reflect the state of the world at the point of which the 
%                      AStar search is ebign conducted.
% Returns:

% via_paths : cell array of via paths

via_paths = {};
for waypoint_idx=1:(size(AStarWaypoints,1)-1)
    waypoints = AstarSearch(AStarWaypoints(waypoint_idx,:), AStarWaypoints(waypoint_idx+1,:), squeeze(occupancyGridVect(waypoint_idx,:,:,:)) );
    vias = zeros( size(waypoints, 1)+1, 4 );
    for i=1:size(waypoints, 1)
        ikResult = inverseKin2( waypoints(i,1), waypoints(i,2), waypoints(i,3), waypoints(i,4), true );
        ikResult2 = inverseKinDynamixel2(ikResult);
        vias(i+1, :) = ikResult2(1:4);
    end

    via_paths(end+1) = {vias};
end


end