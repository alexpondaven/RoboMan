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

    % Check if start is valid
    if occupancyGrid(startIdx(1), startIdx(2), startIdx(3), startIdx(4)) == 1
        disp('[A*] Start location is occupied. Returning')
        waypoints = -1;
        return
    end

    % Check if goal is valid
    if occupancyGrid(startIdx(1), startIdx(2), startIdx(3), startIdx(4)) == 1
        disp('[A*] Goal location is occupied. Returning')
        waypoints = -1;
        return
    end

    % Check if goal == start
    if all( startIdx == goalIdx )
        disp('[A*] Start location and goal location are the same. Returning')
        waypoints = [OGToCartesianCoords(startIdx)];
        return
    end

    % Create closed list
    closedList = zeros(size(occupancyGrid), 'uint8');

    % Create data structure to hold cell information
    % these have a structure defined by
    % cellInfo.parentCoordinates = (theta_g, theta_1, x', y')
    % cellInfo.fCost = scalar f-cost of reaching that particular goal
    % cellInfo.gCost = distance from start to current cell
    INITIAL_F_COST = 1e20; % Initialize an arbitrarily large f-cost
    cellDetails = zeros( size(occupancyGrid) );
    cellInfo.parentCoordinates = [-1, -1, -1, -1];
    cellInfo.fCost = INITIAL_F_COST;
    cellInfo.gCost = 0;
    for tg_idx=1:N_THETA_G_LIST
        for t1_idx=1:N_THETA_1_LIST
            for x_idx=1:N_X_LIST
                for y_idx=1:N_Y_LIST
                    cellDetails(tg_idx, t1_idx, x_idx, y_idx) = cellInfo;
                end
            end
        end
    end
    
    % Initialise starting element for traceback
    cellDetails( startIdx(1), startIdx(2), startIdx(3), startIdx(4) ) = struct( ...
        'parentCoordinates', startIdx, ...
        'fCost', 0, 'gCost', 0);

    % Create openList of cellInfo structs.
    % It will be sorted based on the fCost of each cellInfo variable.
    startElement = struct( "fCost", 0, "coordinates", startIdx );
    openList = [ startElement ];

    while( ~isempty(openList) )

        % Sort openList to get the node with the lowest f-cost
        % We do this by converting the array of structs into a table and sorting it.
        openList = table2struct( sortrows( struct2table(openList), 'fCost' ) );

        % Let the node with the lowest f-cost be our current node, p
        p = openList(1);
        p_theta_g = p.coordinates(1);
        p_theta_1 = p.coordinates(2);
        p_x = p.coordinates(3);
        p_y = p.coordinates(4);
        openList = openList(2:end);     % pop off first element of openList

        % Add this vertex to the closedList
        closedList( p_theta_g, p_theta_1, p_x, p_y ) = 1;

        % Iterate over possible successors of p
        for theta_g_change=-1:1
            for theta_1_change=-1:1
                for x_change=-1:1
                    for y_change=-1:1
                        % d) for each successor
                        curr_theta_g = p_theta_g + theta_g_change;
                        curr_theta_1 = p_theta_1 + theta_1_change;
                        curr_x = p_x + x_change;
                        curr_y = p_y + y_change;
                        
                        % [VALID_IF] If check that successor is valid and not the same as original
                        if  ~(curr_theta_g==p_theta_g && curr_theta_1==p_theta_1 ...
                            && curr_x==p_x && curr_y==p_y) && ...
                            ( occupancyGrid(curr_theta_g, curr_theta_1, curr_x, curr_y)==0 )
                            
                            % [GOAL_IF] i) if successor is the goal, stop search
                            if (curr_theta_g==goalIdx(1) && curr_theta_1==goalIdx(2) ...
                                && curr_x==goalIdx(3) && curr_y==goalIdx(4))

                                disp('[A*] Found goal.')

                                % Indices of waypoints
                                waypointIdx = [ [curr_theta_g, curr_theta_1, curr_x, curr_y]; ...
                                                [p_theta_g, p_theta_1, p_x, p_y] ];

                                % Traverse the path back and append to waypointIdx
                                curr_index = [p_theta_g, p_theta_1, p_x, p_y];
                                parent_coords = cellDetails(curr_index(1), curr_index(2), ...
                                    curr_index(3), curr_index(4)).parentCoordinates;

                                % Find start point by checking that its parent is the same as itself.
                                % Compare element-wise if curr_index==parent_coords
                                while( ~(all( curr_index==parent_coords ))  )
                                    waypointIdx(end+1, :) = parent_coords;     % append
                                    
                                    curr_index = parent_coords;
                                    parent_coords = cellDetails(curr_index(1), curr_index(2), ...
                                        curr_index(3), curr_index(4)).parentCoordinates;
                                end
                                % Convert grid coordinates found in goal to cartesian coordinates                                waypoints = zeros(4, size(waypointIdx,2));
                                for idx=1:size(waypointIdx,1)
                                    waypoints(idx,:) = OGToCartesianCoords(waypointIdx(idx,:));
                                end
                                
                                fprintf('[A*] Goal has %d elements.\n\n', size(waypoints, 1))

                                % invert waypoint list (to be in order)
                                waypoints = flip(waypoints, 1);

                                return
                            end
                            % [GOAL_IF] end goal checking
                            
                            % [ADD_TO_OPEN] if cell is already on closed list, ignore it (it's been searched alr)
                            if ~(closedList(curr_theta_g, curr_theta_1, curr_x, curr_y) == 1)
                                
                                % ii) else, compute both g and h for successor
                                % successor.g = q.g + distance between successor and q
                                curr_g = cellDetails( p_theta_g, p_theta_1, p_x, p_y ).gCost ...
                                + rssq([theta_g_change, theta_1_change, x_change, y_change]);
                                
                                % successor.h = distance from goal to successor using heuristic
                                % We use a straight-line heuristic.
                                theta_g_dist = curr_theta_g - goalIdx(1);
                                theta_1_dist = curr_theta_1 - goalIdx(2);
                                x_dist = curr_x - goalIdx(3);
                                y_dist = curr_y - goalIdx(4);
                                curr_h = rssq([theta_g_dist, theta_1_dist, x_dist, y_dist]);
                                
                                % successor.f = successor.g + successor.h
                                curr_f = curr_g + curr_h;
                                
                                % iii) if the cell hasn't been expanded yet or has a 
                                % higher f-cost than currently, update it
                                % And add to the open list
                                existing_f = cellDetails(curr_theta_g, curr_theta_1, curr_x, curr_y).fCost;
                                if existing_f == INITIAL_F_COST || existing_f > curr_f 
                                    cellInfo.parentCoordinates = [p_theta_g, p_theta_1, p_x, p_y];
                                    cellInfo.fCost = curr_f;
                                    cellInfo.gCost = curr_g;

                                    cellDetails(curr_theta_g, curr_theta_1, curr_x, curr_y) = cellInfo;
                                    openList(end+1) = struct( "fCost", curr_f, ...
                                        "coordinates", [curr_theta_g, curr_theta_1, curr_x, curr_y] );
                                end
                            end
                            % [ADD_TO_OPEN] 
                        end
                        % [VALID_IF] end checking for validity

                    end
                    % end (for loops)
                end
            end
        end
        % e) push q on the closed list
        
    end     % end (while loop)

    
    disp('[A*] did not find a path to the solution.')
    waypoints = -1;
    
end