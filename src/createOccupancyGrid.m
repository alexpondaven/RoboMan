function occupancyGrid = createOccupancyGrid(cubeLocs, cubeHolderLocs)
    % createOccupancyGrid
    % Args:
    % cubeLocs : a 2D array containing the (i,j,height) values for each
    % cube.
    % cubeHolderLocs : a 2D array containing the (i,j) values for each
    % cube holder.
    % Returns:
    % occupancyGrid : a 4D uint8 matrix indexed by 
    % (theta_g, theta_1, x', y') where 1 indicates a particular location 
    % has been occupied.
    
    %% Function parameters
    ogParams = getOccupancyGridParams();

    THETA_G_LIST = ogParams.THETA_G_LIST;
    THETA_1_LIST = ogParams.THETA_1_LIST;
    X_LIST = ogParams.X_LIST;
    Y_LIST = ogParams.Y_LIST;
    XY_SCALING = ogParams.XY_SCALING;
    CUBE_DIM = ogParams.CUBE_DIM;
    HOLDER_HEIGHT = ogParams.HOLDER_HEIGHT;
    HOLDER_DIM = ogParams.HOLDER_DIM;

    occupancyGrid = zeros( ...
        length(THETA_G_LIST), ...
        length(THETA_1_LIST), ...
        length(X_LIST), ...
        length(Y_LIST), ...
        'uint8'...
    );

    %% Create occupancy grid
   
    for theta_1_idx = 1:length(THETA_1_LIST)
        theta_1 = deg2rad(THETA_1_LIST(theta_1_idx));
        for y_idx=1:length(Y_LIST)
            y = Y_LIST(y_idx);
            for x_idx = 1:length(X_LIST)
                x = X_LIST(x_idx);

                % Iterate over cubes and holders to check if they will
                % collide with anything
                % If the current point is contained within the bounding box
                % for a cube or a cube holder, we consider it occupied
                for cube=cubeLocs
                    % Translate cube center ij into x', y' and theta1
                    cube_xPrime = hypot( cube(1)*XY_SCALING, cube(2)*XY_SCALING );
                    cube_yPrime = (0.5 + cube(3)) * CUBE_DIM + HOLDER_HEIGHT;
                    cube_theta1 = atan2(cube(2), cube(1));

                    % Upper and lower angular bounds for each cube
                    cube_theta_bound = atan2(CUBE_DIM/2, (cube_xPrime-CUBE_DIM/2)); 
                    % look at Task2>Cube Manipulation for diagram
                    cube_theta1_ub = cube_theta1 + cube_theta_bound;
                    cube_theta1_lb = cube_theta1 - cube_theta_bound;

                    if ((cube_xPrime+CUBE_DIM/2 >= x) && (cube_xPrime-CUBE_DIM/2 <= x)) && ...
                       ((cube_yPrime+CUBE_DIM/2 >= y) && (cube_yPrime-CUBE_DIM/2 <= y)) && ...
                       ((cube_theta1_ub >= theta_1)   && (cube_theta1_lb <= theta_1)  )

%                         fprintf("cube ub: %0.2f | cube lb: %0.2f\n", rad2deg(cube_theta1_ub), rad2deg(cube_theta1_lb));
%                         fprintf("cube at x': %0.2f y': %0.2f theta1: %0.2f (i:%d j: %d) collides\n", ...
%                             cube_xPrime, cube_yPrime, rad2deg(cube_theta1), cube(1), cube(2));
%                         fprintf("with current coordinates, x: %0.2f, y: %0.2f, theta1: %0.2f\n", ...
%                             x, y, rad2deg(theta_1));
%                         fprintf("Writing to occupancy grid indexes: %d %d %d\n\n", ...
%                             theta_1_idx, y_idx, x_idx );
                        
                        for theta_g_idx = 1:length(THETA_G_LIST)
                            occupancyGrid(theta_g_idx, theta_1_idx, x_idx, y_idx) = 1;
                        end
                    end
                end

                for holder=cubeHolderLocs
                    % Translate cube ij into xy
                    holder_xPrime = hypot( holder(1)*XY_SCALING, holder(2)*XY_SCALING );
                    holder_theta1 = atan2(holder(2), holder(1));

                    % Upper and lower angular bounds for holder
                    holder_theta_bound = atan2(HOLDER_DIM/2, (holder_xPrime-HOLDER_DIM/2)); 
                    % look at Task2>Cube Manipulation for diagram
                    holder_theta1_ub = holder_theta1 + holder_theta_bound;
                    holder_theta1_lb = holder_theta1 - holder_theta_bound;

                    if ((holder_xPrime+HOLDER_DIM/2 >= x) && (holder_xPrime-HOLDER_DIM/2 <= x)) && ...
                       ((y >= 0) && (y <= HOLDER_HEIGHT)) && ...
                       ((holder_theta1_ub >= theta_1)   && (holder_theta1_lb <= theta_1)  )

%                         fprintf("holder ub: %0.2f | lb: %0.2f\n", rad2deg(holder_theta1_ub), rad2deg(holder_theta1_lb));
%                         fprintf("holder at x': %0.2f theta1: %0.2f (i:%d j: %d) collides\n", ...
%                             holder_xPrime, rad2deg(holder_theta1), holder(1), holder(2));
%                         fprintf("with current coordinates, x: %0.2f, y: %0.2f, theta1: %0.2f\n", ...
%                             x, y, rad2deg(theta_1));
%                         fprintf("Writing to occupancy grid indexes %d %d %d\n\n", ...
%                             theta_1_idx, y_idx, x_idx );

                        for theta_g_idx = 1:length(THETA_G_LIST)
                            occupancyGrid(theta_g_idx, theta_1_idx, x_idx, y_idx) = 1;
                        end
                    end
                end

                % Check for admissibility
                % Do IK here. If the current position cannot be
                % reached then mark the occupancy grid position
                % accordingly.
                for theta_g_idx = 1:length(THETA_G_LIST)
                    theta_g = deg2rad(THETA_G_LIST(theta_g_idx));
                    % convert current x', y', theta1 value back to x, y, z
                    xVal = x * cos(theta_1);
                    yVal = x * sin(theta_1);
                    zVal = y;
                    [~, ec] = inverseKin2(xVal, yVal, zVal, theta_g, false);
                    if ec ~= 0
                        occupancyGrid(theta_g_idx, theta_1_idx, x_idx, y_idx) = 1;

%                         fprintf("Position x: %0.2f y: %0.2f z: %0.2f unreachable with theta_g: %0.2f\n", ...
%                             xVal, yVal, zVal, rad2deg(theta_g) );
%                         fprintf("Writing to occupancy grid index %d %d %d %d\n\n", ...
%                             theta_g_idx, theta_1_idx, y_idx, x_idx );
                    end
                end
            end
        end
    end

end