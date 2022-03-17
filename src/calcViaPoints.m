function theta_vias = calcViaPoints(start_waypoint, goal_waypoint, occupancyGrid)
% Wrapper around AStarSearch and InverseKinDynamixel2. 
% search for a path in cartesian space and convert it to joint space.
% Args:
% start_waypoint     : Starting waypoint (x,y,z,theta_g) of end effector
% goal_waypoint      : End waypoint (x,y,z,theta_g) of end effector
% occupancyGrid      : occupancy grid for A* search
%
% Returns:
% theta_vias : via points of servo1-4 angles

    waypoints = AstarSearch(start_waypoint, goal_waypoint, occupancyGrid );
    theta_vias = zeros( size(waypoints, 1), 4 );
    for i=1:size(waypoints, 1)
        % Note: Gripper open/close not important here as only theta1..4 taken
        ikResult = inverseKinDynamixel2( waypoints(i,1), waypoints(i,2), waypoints(i,3), waypoints(i,4), true ); 
        theta_vias(i, :) = ikResult(1:4);
    end

end