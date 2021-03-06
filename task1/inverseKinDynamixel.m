function theta = inverseKinDynamixel(x,y,z,theta_G, gripSize)
    % Wrapper around inverseKin to convert angles to Dynamixel ticks

    % Hip
    D11_PARAMS = struct('LB',deg2rad(90), ...
                 'UB', deg2rad(270), ...
                 'invert', 1, ...
                 'offset', pi);
    % Shoulder
    D12_PARAMS = struct('LB',deg2rad(65), ...
                 'UB', deg2rad(280), ...
                 'invert', -1, ...
                 'offset', pi);
    % Elbow
    D13_PARAMS = struct('LB',deg2rad(65), ...
                 'UB', deg2rad(265), ...
                 'invert', -1, ...
                 'offset', pi);

    
    % Wrist
    D14_PARAMS = struct('LB',deg2rad(80), ...
                 'UB', deg2rad(300), ...
                 'invert', -1, ...
                 'offset', pi);
    
    % Gripper
    D15_PARAMS = struct('LB',deg2rad(90), ...       % wide open
                 'UB', deg2rad(232), ...            % fully closed
                 'invert', 1, ...
                 'offset', 0);

    theta = inverseKin(x,y,z, theta_G, gripSize);
    
    PARAMS = [D11_PARAMS, D12_PARAMS, D13_PARAMS, D14_PARAMS, D15_PARAMS];
    % modify theta
    for i=1:5
        % Modify theta
        theta(i) = theta(i) * PARAMS(i).invert;     % invert if necessary (for shoulder, wrist, elbow)
        theta(i) = theta(i) + PARAMS(i).offset;

        % Make sure theta is in range described by PARAMS
        if theta(i) > PARAMS(i).UB
            fprintf("Angle %04d of servo %d over upper bound\n", theta(i), i)
            
            theta(i) = PARAMS(i).UB;
        end
        if theta(i) < PARAMS(i).LB
            fprintf("Angle %04d of servo %d under lower bound\n", theta(i), i)
            
            theta(i) = PARAMS(i).LB;
        end
        
        % Convert to ticks
        theta(i) = theta(i) * 2048/pi;
    end


end
