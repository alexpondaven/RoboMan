function waypoints = AstarSearch( startPos, endPos, occupancyGrid )
    % Finds the shortest collision-free path from startPos to endPos through
    % the occupancy grid.
    % 
    % Args:
    % startPos      : The starting position (x, y, z, theta_g) of the end effector.
    % endPos        : The goal position (x, y, z, theta_g) of the end effector.
    % occupancyGrid : 3D array of (theta_1, x', y') configurations
    % 
    % Returns:
    % waypoints : 2D array of (x, y, z, theta_g) angles in radians.
    
    ogParams = getOccupancyGridParams();
    N_X_LIST = size(ogParams.X_LIST, 2);
    N_Y_LIST = size(ogParams.Y_LIST, 2);
    % N_THETA_G_LIST = size(ogParams.THETA_G_LIST, 2);
    N_THETA_1_LIST = size(ogParams.THETA_1_LIST, 2);

    %% Convert start and end positions to grid coordinates
    startIdx = cartesianToOGCoords(startPos)
    goalIdx = cartesianToOGCoords(endPos)

    %% Perform A* Search

    % Check if start is valid
    if occupancyGrid(startIdx(2), startIdx(3), startIdx(4)) == 1 || ~checkOGAdmissibility(startIdx) || ~ogIsValid(startIdx)
        disp('[A*] Start location is occupied or invalid. Returning')
        waypoints = -1;
        return
    end

    % Check if goal is valid
    if occupancyGrid(goalIdx(2), goalIdx(3), goalIdx(4)) == 1 || ~checkOGAdmissibility(goalIdx) || ~ogIsValid(goalIdx)
        disp('[A*] Goal location is occupied or invalid. Returning')
        waypoints = -1;
        return
    end

    % Check if goal == start
    if all( startIdx == goalIdx )
        disp('[A*] Start location and goal location are the same. Returning')
        waypoints(1,:) = OGToCartesianCoords(startIdx);
        return
    end

    % Create closed list
    closedList = zeros([N_THETA_1_LIST, N_X_LIST, N_Y_LIST], 'uint8');

    % Create data structure to hold cell information
    % these have a structure defined by
    % cellInfo.parentCoordinates = (theta_g, theta_1, x', y')
    % cellInfo.fCost = scalar f-cost of reaching that particular goal
    % cellInfo.gCost = distance from start to current cell
    INITIAL_F_COST = 1e20; % Initialize an arbitrarily large f-cost
    cellDetails.parentCoordinates = [];
    cellDetails.fCost = [];
    cellDetails.gCost = [];
    
    for t1_idx=1:N_THETA_1_LIST
        for x_idx=1:N_X_LIST
            for y_idx=1:N_Y_LIST
                cellDetails.parentCoordinates(t1_idx, x_idx, y_idx, :) = [-1, -1, -1];
                cellDetails.fCost(t1_idx, x_idx, y_idx) = INITIAL_F_COST;
                cellDetails.gCost(t1_idx, x_idx, y_idx) = 0;
            end
        end
    end
    
    % Initialise starting element for traceback
    cellDetails.parentCoordinates( startIdx(2), startIdx(3), startIdx(4), : ) ...
        = [startIdx(2), startIdx(3), startIdx(4)];
    cellDetails.fCost( startIdx(2), startIdx(3), startIdx(4) ) = 0;
    cellDetails.gCost( startIdx(2), startIdx(3), startIdx(4) ) = 0;

    % Create openList of cellInfo structs.
    % It will be sorted based on the fCost of each cellInfo variable.
    % startElement = struct( "fCost", 0, "coordinates", startIdx );
    openList.fCost(1) = 0;
    openList.coordinates(1,:) = startIdx(2:4);
    elemsInserted = false;      % Keep track of whether we need to sort openList again

    while( ~isempty(openList) )

        % Sort openList to get the node with the lowest f-cost
        % We do this by converting the array of structs into a table and sorting it.
        if length(openList.fCost) > 1 && elemsInserted
            [openList.fCost, sortIdx] = sort(openList.fCost);
            openList.coordinates = openList.coordinates(sortIdx, :);
            % openList = table2struct( sortrows( struct2table(openList), 'fCost' ) );
        end

        % Let the node with the lowest f-cost be our current node, p
        p = openList.coordinates(1,:);
        p_theta_1 = p(1);
        p_x = p(2);
        p_y = p(3);
        % pop off first element of openList
        openList.fCost = openList.fCost(2:end);
        openList.coordinates = openList.coordinates(2:end, :);
        elemsInserted = false;

        % Add this vertex to the closedList
        closedList( p_theta_1, p_x, p_y ) = 1;

        % Iterate over possible successors of p

        for theta_1_change=-1:1
            for x_change=-1:1
                for y_change=-1:1
                    % d) for each successor
                    % curr_theta_g = p_theta_g + theta_g_change;
                    curr_theta_1 = p_theta_1 + theta_1_change;
                    curr_x = p_x + x_change;
                    curr_y = p_y + y_change;
                    
                    % [VALID_IF] If check that successor is valid and not the same as original
                    if ogIsValid([1, curr_theta_1, curr_x, curr_y])  % bounds checking
                        
                        % Check if the node is reachable. (or has been proven to be unreachable)
                        if closedList(curr_theta_1, curr_x, curr_y) ~= 2 && ...
                            ~( curr_theta_1==p_theta_1 ...
                            && curr_x==p_x && curr_y==p_y) && ...
                            ( occupancyGrid(curr_theta_1, curr_x, curr_y)==0 )

                                
                            % [GOAL_IF] i) if successor is the goal, stop search
                            if (curr_theta_1==goalIdx(2) ...
                                && curr_x==goalIdx(3) && curr_y==goalIdx(4))

                                disp('[A*] Found goal.')

                                % Indices of waypoints
                                waypointIdx = [ [curr_theta_1, curr_x, curr_y]; ...
                                                [p_theta_1, p_x, p_y] ];

                                % Traverse the path back and append to waypointIdx
                                curr_index = [p_theta_1, p_x, p_y];
                                parent_coords = cellDetails.parentCoordinates( ...
                                    curr_index(1), ...
                                    curr_index(2), curr_index(3), : );

                                % Find start point by checking that its parent is the same as itself.
                                % Compare element-wise if curr_index==parent_coords
                                while( ~(all( curr_index==parent_coords ))  )
                                    waypointIdx(end+1, :) = parent_coords;                                    
                                    curr_index = parent_coords;

                                    parent_coords = cellDetails.parentCoordinates( ...
                                        curr_index(1), curr_index(2), ...
                                        curr_index(3), : );

                                end
                                % Convert grid coordinates found in goal to cartesian coordinates
                                waypoints = zeros(size(waypointIdx,2), 4);
                                t_g_vect = linearInterpolate(endPos(4), startPos(4), size(waypointIdx,1));

                                for idx=1:size(waypointIdx,1)
                                    waypoints(idx,:) = OGToCartesianCoords([1 waypointIdx(idx,:)]);
                                    waypoints(idx,4) = t_g_vect(idx);
                                end
                                % invert waypoint list (to be in order)
                                waypoints = flip(waypoints, 1);
                                fprintf('[A*] Goal bef t_g augmentation has %d elements.\n', size(waypoints, 1))

                                % We get a 'complete' set of waypoints here, now we need to interpolate for theta_g
                                % [x,y,z,thetag] rows
                                theta_g_diff = diff(waypoints(:,4))';    % convert into a row vector
                                diff_idxs = find(theta_g_diff);  % Indices of waypoints where there was a change in theta_g
                                interp_waypoints = [];
                                
                                for idx=1:size(waypoints,1)
                                    % fprintf("curr_idx = %d ", idx);
                                    if any(idx==diff_idxs)
                                        interp_start = waypoints(idx, :);
                                        interp_end = waypoints(idx+1, :);
                                        NUM_INTERPOLATE = ceil( abs(theta_g_diff(idx)) / (pi/2) * 15) + 1;
                                        % fprintf("with NUM_INTERPOLATE = %d\n", NUM_INTERPOLATE);
                                        % a rotation of 90 degrees should give us 15 intermediate waypts (arbitrarily)
                                        % +1 to at least have 2 waypoints
                                        interpolated_waypoints = linearInterpolate(interp_start, interp_end, NUM_INTERPOLATE);
                                        interp_waypoints = [interp_waypoints;
                                                            interpolated_waypoints(1:end-1,:)];   % end will be added below in the next iteration
                                    else
                                        % fprintf("\n");
                                        interp_waypoints = [interp_waypoints;
                                                            waypoints(idx,:)];
                                    end
                                end
                                
                                fprintf('[A*] Goal after t_g augmentation has %d elements.\n', size(interp_waypoints, 1))
                                waypoints = interp_waypoints;

                                return
                            end
                            % [GOAL_IF] end goal checking
                            
                            % [ADD_TO_OPEN] if cell is already on closed list, ignore it (it's been searched alr)
                            if ~(closedList(curr_theta_1, curr_x, curr_y) == 1)
                                
                                % ii) else, compute both g and h for successor
                                % successor.g = q.g + distance between successor and q
                                curr_g = cellDetails.gCost( p_theta_1, p_x, p_y ) ...
                                + sqrt( sum( [theta_1_change, x_change, y_change].^2 ));

                                
                                % successor.h = distance from goal to successor using heuristic
                                % We use a straight-line heuristic.
                                theta_1_dist = curr_theta_1 - goalIdx(2);
                                x_dist = curr_x - goalIdx(3);
                                y_dist = curr_y - goalIdx(4);
                                curr_h = sqrt( sum( [theta_1_dist, x_dist, y_dist].^2 ));
                                
                                % successor.f = successor.g + successor.h
                                curr_f = curr_g + curr_h;
                                
                                % iii) if the cell hasn't been expanded yet or has a 
                                % higher f-cost than currently, update it
                                % And add to the open list
                                existing_f = cellDetails.fCost(curr_theta_1, curr_x, curr_y);
                                if existing_f == INITIAL_F_COST || existing_f >= curr_f 
                                    cellDetails.parentCoordinates(...
                                        curr_theta_1, curr_x, curr_y, :) = ...
                                        [p_theta_1, p_x, p_y];
                                    cellDetails.fCost(...
                                        curr_theta_1, curr_x, curr_y) = curr_f;
                                    cellDetails.gCost(...
                                        curr_theta_1, curr_x, curr_y) = curr_g;
                                    
                                    openList.fCost(end+1) = curr_f;
                                    openList.coordinates(end+1,:) = [curr_theta_1, curr_x, curr_y];
                                    
                                    % fprintf("Inserted idx [%d %d %d %d]\n", curr_theta_g, curr_theta_1, curr_x, curr_y);
                                    elemsInserted = true;
                                end
                            end
                            % [ADD_TO_OPEN] 
                        end
                    end
                    % [VALID_IF] end checking for validity
                end
                % end (for loops)
            end
        end
        % e) push q on the closed list
        
    end     % end (while loop)

    
    disp('[A*] did not find a path to the solution.')
    waypoints = -1;
        
end