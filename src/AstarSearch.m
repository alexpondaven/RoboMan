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
    N_X_LIST = size(ogParams.X_LIST, 2);
    N_Y_LIST = size(ogParams.Y_LIST, 2);
    N_THETA_G_LIST = size(ogParams.THETA_G_LIST, 2);
    N_THETA_1_LIST = size(ogParams.THETA_1_LIST, 2);

    %% Convert start and end positions to grid coordinates
    startIdx = cartesianToOGCoords(startPos);
    goalIdx = cartesianToOGCoords(endPos);

    %% Perform A* Search
    % TODO

    % ~ Check if start is valid
    % ~ Check if goal is valid

    % ~ Check if goal == start

    % Create closed list
    closedList = zeros(size(occupancyGrid), 'uint8');

    % Create data structure to hold cell information
    % these have a structure defined by
    % cellInfo.parentCoordinates = (theta_g, theta_1, x', y')
    % cellInfo.fCost = scalar f-cost of reaching that particular goal
    cellDetails = zeros( size(occupancyGrid) );
    cellInfo.parentCoordinates = [-1, -1, -1, -1];
    cellInfo.fCost = 1e20;
    % Initialize elements of cellDetails to have an arbitrarily large f-cost
    for tg_idx=1:N_THETA_G_LIST
        for t1_idx=1:N_THETA_1_LIST
            for x_idx=1:N_X_LIST
                for y_idx=1:N_Y_LIST
                    cellDetails(tg_idx, t1_idx, x_idx, y_idx) = cellInfo;
                end
            end
        end
    end

    % Create openList of cellInfo structs.
    % It will be sorted based on the fCost of each cellInfo variable.
    startElement = struct( "fCost", 0, "coordinates", startIdx );
    openList = [ startElement ];

    foundDest = false;

    while( ~isempty(openList) )

        % Sort openList to get the node with the lowest f-cost
        % We do this by converting the array of structs into a table and sorting it.
        openList = table2struct( sortrows( struct2table(openList), 'fCost' ) );

        % Let the node with the lowest f-cost be our current node, p
        p = openList(1);
        openList = openList(2:end);     % pop off first element of openList

        % Add this vertex to the closedList
        closedList( p.coordinates(1), ...
                    p.coordinates(2), ...
                    p.coordinates(3), ...
                    p.coordinates(4) ...
            ) = 1;

        % Iterate over possible successors of p
            % d) for each successor
                % i) if successor is the goal, stop search
                
                % ii) else, compute both g and h for successor
                    % successor.g = q.g + distance between successor and q
                    % successor.h = distance from goal to successor using heuristic

                    % successor.f = successor.g + successor.h

                % iii) if a node with the same position as successor is in the OPEN 
                % list with a lower f than successor, skip this successor
        
                % iv) if a node with the same position as successor is in the CLOSED 
                % list with a lower f than successor, skip this successor
            
                % otherwise, add  the node to the open list
                
            % end (for loop)
    
        % e) push q on the closed list
        
    end     % end (while loop)

    %% Convert grid coordinates found in goal to cartesian coordinates
    waypoints = zeros(4, size(waypointIdx,2));
    for idx=1:size(waypointIdx,2)
        waypoints(:,idx) = OGToCartesianCoords(waypointIdx(:,idx));
    end

end