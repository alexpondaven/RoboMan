function visualisePath(waypoints, path)
% Visualises path planned by A* algorithm through a set of waypoints, ad infinitum.
%
% Args:
% waypoints : vector of (x,y,z,theta_g) from A*
% path      : vector of theta values generated from IK on each point on the path
% 
% Does not return anything.

    REPEAT_TIMES = 5;
    PAUSE_DURATION = 0.4;

    % Repeat animation 5x
    for REPEAT=1:REPEAT_TIMES
        for i=1:size(waypoints,1)

            plot3(1,1,1)
            hold on

            % Get joint positions
            joints = getJointPositions(path(i,:));
            
            % Draw world frame
            plot3([0,250], [0,  0], [0,  0], 'r', 'LineWidth', 1.0)
            plot3([0,  0], [0,250], [0,  0], 'g', 'LineWidth', 1.0)
            plot3([0,  0], [0,  0], [0,250], 'b', 'LineWidth', 1.0)
            plot3(waypoints(1,1), waypoints(1,2), waypoints(1,3), 'gx', 'LineWidth', 2.0)
            plot3(waypoints(end,1), waypoints(end,2), waypoints(end,3), 'rx', 'LineWidth', 2.0)
            axis([-100, 250, -250, 250, 0, 250])


            for visIdx=2:size(joints,2)
                drawLine(joints(:,visIdx-1), joints(:,visIdx));
                % drawFrame(joints(:,visIdx), 20);
            end

            axis([-50, 250, -250, 250, 0, 250])
            plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'kx-')
            grid on
            hold off
           
            pause(PAUSE_DURATION);
        end
    end

end

